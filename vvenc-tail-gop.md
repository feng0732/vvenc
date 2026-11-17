# vvenc 短序列尾部 GOP 处理分析

> 分析对象：本仓库源码（分支 `task/22`，基线提交 `58cbc54`）。全部结论仅依据代码与仓库内既有材料（配置文件、单元测试、头文件注释），不引入外部假设。文中引用均给出 `文件:行号`。
>
> 术语约定：vvenc 代码中没有 "tail" 这一命名，序列尾部在实现里对应 **flush（冲刷）** 流程，由 `Picture::isFlush` 标志贯穿整条编码流水线（`source/Lib/CommonLib/Picture.h:236`）。"尾段" 指 flush 时仍留在各阶段队列里、尚不足一个完整 GOP 的图片集合。

---

## 0. 背景：编码顺序、显示顺序与 GOP 配置的交汇点

vvenc 的随机访问结构（默认 `GOPSize=32`，见 `source/Lib/vvenc/vvencCfg.cpp:384` 与 `cfg/gop32.cfg`）是分层 B 帧结构，两套顺序同时存在：

- **显示顺序**：即 POC 顺序，输入 YUV 按 POC 递增到达。`GOPEntry::m_POC` 是 GOP 内的显示位置（`cfg/gop32.cfg` 中 `Frame1..Frame32` 的第二列）。
- **编码/解码顺序**：即 `GOPEntry::m_codingNum` 顺序（`source/Lib/CommonLib/TypeDef.h:870`）。dyadic 结构中 TL0 关键帧先编码，然后逐层向下。GOP-32 的编码顺序为 POC 32 → 16 → 8 → 24 → 4 → 12 → 20 → 28 → …，与 `cfg/gop32.cfg` 的行排列一致（行序即编码序）。

两套顺序的交汇由三个组件完成：

1. `GOPCfg::getNextGopEntry` 在每张输入图到达时为其绑定 `GOPEntry`，写入 POC、`m_codingNum`、`m_gopNum`、参考增量 POC（`source/Lib/EncoderLib/GOPCfg.cpp:127-241`）。
2. `EncStage::addPicSorted` 在非预取阶段按 `m_codingNum` 把图片插入处理队列（`source/Lib/EncoderLib/EncStage.h:351-359`），使队列天然按编码序排列。
3. `EncGOP::xInitPicsInCodingOrder` 要求 `m_codingNum == m_lastCodingNum + 1` 严格连续才初始化下一张（`source/Lib/EncoderLib/EncGOP.cpp:1664`），而 AU 输出队列 `m_gopEncListOutput` 也按同一顺序压入、从队首写出（`EncGOP.cpp:1678`、`555-558`），因此码流 AU 顺序等于编码顺序；显示顺序通过每幅图的 POC/CTS 与 DTS 偏移恢复（`EncGOP.cpp:2530-2543`）。

---

## 1. 参考图像选择如何与编码顺序、显示顺序在尾部交汇，终止信号如何避免过期参考

### 1.1 参考关系在配置期就是"按编码序可用"构造的

`GOPCfg::xCreateGopList` 构造 GOP 内参考关系时，维护一个按 POC 索引的 `availList`，只有**已经在编码序上出现过**的条目才能作为参考候选：

- `xAddRefPicsBckwd` 从 `m_POC-1` 向下扫描已可用条目（`GOPCfg.cpp:1033-1047`）；
- `xAddRefPicsFwd` 从 `m_POC+1` 向上扫描，但这些"前向"参考（显示顺序上在未来的帧，如 POC 16 参考 POC 32）之所以可用，是因为它们在编码序上更早（POC 32 是 GOP 的第一张编码图），已先进入 `availList`（`GOPCfg.cpp:1049-1063`、`501`）。

这就是编码顺序与显示顺序在参考选择上的根本交汇点：**L1 前向参考在显示序上属于"未来"，但在编码序上属于"过去"**。尾部的问题正源于此——最后一个 GOP 的 POC 32（下一 GOP 的 TL0 锚点）在序列结束时可能根本不存在。

此外，`xCreateGopList` 末尾会反向遍历 GOP，把"后续图片需要存活"的参考追加进各条目的 RPL（`GOPCfg.cpp:529-582`），并构造一个 POC 为 `2*maxGopSize` 的假条目 `e2` 来保留下一 GOP 所需参考；当且仅当这是最后一个被裁剪的 GOP（`bIsLast = m_maxGopSize != gopSize`，`GOPCfg.cpp:514`）时，`e2` 不带任何参考需求——这是配置期对"尾部无下一 GOP"的第一道边界处理。

### 1.2 flush 终止信号的产生与传播

公共 API 以空 YUV 缓冲表达终止：`vvenc_encode(enc, NULL, ...)` 使 `VVEncImpl` 进入 `INTERNAL_STATE_FLUSHING`（`source/Lib/vvenc/vvencimpl.cpp:359-361`），随后 `EncLib::encodePicture` 在每次调用中把 `flush=true` 传给所有阶段（`source/Lib/EncoderLib/EncLib.cpp:494-500`）。

`EncStage` 把终止信号固化到图片对象上，规则分两类（`source/Lib/EncoderLib/EncStage.h:379-400`）：

- **阻塞式阶段**（PreProcess、MCTF、CuTree，`m_isNonBlocking=false`）：首次见到 flush 时，把队列中**所有**图片标记 `isFlush=true`，因为它们相对输入是确定的；
- **非阻塞阶段**（最后一级 GOP 编码器）：只保证编码序队尾那张被标记；新插入的 flush 图片还会把它之后（编码序）的图片一并标记，并有 `CHECK(!isInitDone)` 保证这些图片尚未开始编码（`EncStage.h:361-372`）。

`runStage` 的调度条件 `(m_procList.size() >= m_minQueueSize) || (m_procList.size() && flush)`（`EncStage.h:403-404`）使队列在不足最小深度（GOP 编码器要求 `GOPSize+1`，`EncLib.cpp:286`）时也能在 flush 下排空；`m_flushAll` 阶段则在 flush 时循环处理到队列为空（`EncStage.h:439`）。

### 1.3 尾部 GOP 起点的修正：fixStartOfLastGop

PreProcess 第一次在 flush 状态下被调度时（`source/Lib/EncoderLib/PreProcess.cpp:160-173`）：

1. `xGetStartOfLastGop` 排除 lead/trail 图片，按 `m_codingNum` 排序，找出最后一个 `m_gopNum` 的第一张图（`PreProcess.cpp:259-295`）；
2. 调 `GOPCfg::fixStartOfLastGop`（`source/Lib/EncoderLib/GOPCfg.cpp:269-280`）：无条件置 `m_isStartOfGop=true`；若不是 POC0IDR 模式、处于 GOP 0 且原本不是 intra 起点，则进一步提升为 intra 起点——切片类型改 `'I'`、TL 归零、记录 `m_lastIntraPOC`。

这一步的意义：尾部残段在正常轮转中本不是 GOP 边界，但它之后再无图片，必须自带一个不依赖"未来锚点"的起点。提升为 I 切片后，`xInitFirstSlice` 会把它切成 `VVENC_I_SLICE`，L0/L1 参考数归零（`EncGOP.cpp:2125-2128`、`2201-2202`），从根本上消除对不存在图片的参考需求。CuTree 侧的注释也明确记录了这一动态：*"at the end of the sequence fixStartOfLastGop can make a picture the start of an intra period"*（`source/Lib/EncoderLib/CuTree.cpp:617-619`），因此 `xMarkRandomAccess` 每次调用都重读 GOPEntry 而非缓存结论。

### 1.4 避免过期/缺失参考的三道运行时防线

即使 GOPEntry 的模板 RPL 仍指向不存在的 POC，编码初始化时还有三层过滤：

**第一道：缺失检测 + 显式 RPL 重建。** `xInitFirstSlice` 先按 GOPEntry 选 SPS 中的默认 RPL（`EncGOP.cpp:2179`，`xSelectReferencePictureList` 见 `2476-2493`），随后用 `Slice::isRplPicMissing` 逐个 POC 核对实际图片列表（`source/Lib/CommonLib/Slice.cpp:957-1053`）：短期参考必须在 `rcListPic` 中存在、`isReferenced==true` 且不是未来的 IDR_NO_LP 图片（`Slice.cpp:1039`），否则报告缺失。一旦缺失，`createExplicitReferencePictureSetFromReference` 丢弃所有找不到对应 `Picture*` 的 RPL 条目，只保留真实可用的参考，不足时再从对侧列表补齐（`Slice.cpp:1055-1170` 及后续）。IDR/BLA 直接跳过检查，因为 DPB 本就要清空（`Slice.cpp:959`、`1060`）。

**第二道：基于 RPL 的参考标记。** `applyReferencePictureListBasedMarking` 遍历 DPB，把不在最终 RPL 中的已初始化图片标记 `isReferenced=false`（`Slice.cpp:947-952`）；`setDecodingRefreshMarking` 在 IDR 时把除自身外所有图片标记为不可参考（`Slice.cpp:567-585`）。这保证过期图片不会被后续图片引用，也使 `isRplPicMissing` 的 `isReferenced` 条件形成闭环。

**第三道：启动时序约束。** 真正开始编码前，`findNextPicToStart`/`isReady` 要求非 IFP 模式下所有参考已重建（`checkAllRefPicsReconstructed`，`Slice.cpp:445-460`，调用点 `EncGOP.cpp:472-478`）；IFP 放宽为"已开始且可访问"（`checkAllRefPicsAccessible`，`Slice.cpp:462-477`），且 `hasUnstartedRef` 禁止行锁图片先于其参考启动（`EncGOP.cpp:356-371`、`401-405`）。flush 还放宽了 `xInitPicsInCodingOrder` 的编码序连续性检查——注意条件是 `m_codingNum != m_lastCodingNum+1 && !picList.back()->isFlush` 才 break（`EncGOP.cpp:1664`），即终止信号允许队列带着残段直接初始化，而不是死等永远不会到达的下一个 codingNum。

CuTree 在自己的低分辨率分析域内复制了同一套语义。`refState` 对 `refPoc > lastPoc` 的参考区分两种情况：**非 flush 时返回 `NOT_ARRIVED`（稍后重试），flush 时返回 `MISSING`（跳过该条目继续扫描）**（`source/Lib/EncoderLib/CuTree.cpp:697-700`）。其注释明确说明这是在模拟 `createExplicitReferencePictureSetFromReference` 的行为（`CuTree.cpp:684-690`）。批次就绪条件同样是"GOP+lookahead 到齐 **或** flush"（`CuTree.cpp:760-780`）。

### 1.5 码率控制与资源释放对终止信号的响应

- flush 图片启动 GOP 级 RC 时，`setRCRateSavingState(0)` 通知预算估算"视频即将结束"（`EncGOP.cpp:505-508`）；且 RC 不再要求 lookahead 缓存中有后续 POC（条件 `!pic->isFlush` 才 break，`EncGOP.cpp:512`），`processFirstPassData` 直接以 flush 模式处理（`EncGOP.cpp:516`）。
- 图片释放的终态判定 `allPicsDone = m_numPicsCoded >= m_picCount && (picList.empty() || picList.back()->isFlush)`（`EncGOP.cpp:811`）保证 flush 后所有图片最终都会被回收，不依赖 `isReferenced` 状态。
- IFP 确定性 RC 更新也把队首 `isFlush` 作为凑齐窗口的条件之一（`EncGOP.cpp:1711`）。

---

## 2. 完整 GOP、短尾段、单图尾段各自应呈现的参考关系与输出顺序

以下三种形态以默认随机访问配置（`GOPSize=32`、picReordering=1、无固定 intra period 时 `m_IntraPeriod=0`，`vvencCfg.cpp:381`）为基准描述。

### 2.1 完整 GOP（32 张，序列中部或帧数恰为 GOP 整数倍）

- **参考关系**：严格分层。TL0 锚点（POC 为 32 的倍数）参考上一 GOP 的 TL0 锚点；每个高层 B 帧只参考 temporalId 不高于自己的已编码图片（`xAddRefPicsBckwd/Fwd` 的 `refEntry->m_temporalId <= gopEntry->m_temporalId` 条件，`GOPCfg.cpp:1040`、`1056`）。以 GOP-32 为例：POC 16（TL1）双向参考 POC 0/32；POC 8（TL2）参考 POC 0/16；POC 4（TL3）参考 POC 0/8/12/20……与 `cfg/gop32.cfg:7-11` 的 delta POC 列一致。GOP 之间通过 `xGetPrevGopRefs` 把上一 GOP 尾部仍存活的参考传入（`GOPCfg.cpp:602-669`）。
- **输出顺序**：编码序 = AU 输出序，即 32 → 16 → 8 → 24 → 4 → … → 31（`cfg/gop32.cfg` 行序）；显示序由 POC/CTS 还原，DTS 通过 `iDiffFrames - maxTLayer` 的偏移计算（`EncGOP.cpp:2532-2543`）。GOP 编码器正常情况下要求队列深度达到 `GOPSize+1` 才启动（`EncLib.cpp:286`），确保 GOP 内全部参考到齐。
- **NAL 类型**：常规图片为 TRAIL（`EncGOP.cpp:1466`），intra 周期起点按配置为 IDR_W_RADL/IDR_N_LP/CRA（`EncGOP.cpp:1428-1449`），STSA 条件由 `xSetSTSA` 预计算（`GOPCfg.cpp:799-846`）并在 `xIsSliceTemporalSwitchingPoint` 中落地（`EncGOP.cpp:1469` 起）。

### 2.2 短尾段（1 < 残段图片数 < GOPSize，flush 触发）

设序列在两个 TL0 锚点之间结束，残段含 K 张图（1 < K < 32）。

- **参考关系的变化**：
  1. 残段第一张被 `fixStartOfLastGop` 标记为 GOP 起点；若它同时是 GOP 0 且非 POC0IDR，则直接提升为 I 切片（`GOPCfg.cpp:272-279`），此时残段退化为"一个 I + 若干 P/B"，无任何跨 GOP 前向参考。
  2. 若未提升为 I（例如 POC0IDR 模式或不在 GOP 0），模板 RPL 中指向"未来锚点"（如残段最后一张之后的 POC）的条目在 `isRplPicMissing` 处判定缺失，`createExplicitReferencePictureSetFromReference` 将其剔除，列表只保留残段内与前序 GOP 中真实存在且 `isReferenced` 的图片（`Slice.cpp:1029-1050`、`1073-1105`）。CuTree 域内对应行为是 flush 把 `NOT_ARRIVED` 变 `MISSING` 后跳过（`CuTree.cpp:697-700`），单元测试 `"end of sequence"` 用例固定了该期望：flush 后 L1 不可用则退化为单向预测（`test/vvenc_unit_test/vvenc_unit_test.cpp:8495-8497`）。
  3. 残段内的分层关系仍然成立：先编码的低 TL 图片供后编码的高 TL 图片参考，只是层级可能不完整（例如没有 TL1 锚点 POC 16 时，POC 8 只能参考 POC 0）。
- **输出顺序**：残段图片仍按 `m_codingNum` 顺序进入 `m_gopEncListOutput` 并逐张写出（`EncGOP.cpp:1677-1678`、`547-558`）；flush 解除了两处正常等待——队列最小深度（`EncStage.h:403-404`）与 codingNum 连续性（`EncGOP.cpp:1664`），因此残段无需凑齐 32 张即可排空。最后一张 AU 写出后 `m_numPicsCoded == m_picCount`，`xReleasePictures` 的 `allPicsDone` 分支释放全部残留图片（`EncGOP.cpp:809-825`）。
- **DPB 约束不被突破**：尾段使用的裁剪 GOP 列表在初始化期就通过 `xCheckDBPConstraints` 校验（`GOPCfg.cpp:1001-1031`，调用点 `99-106`），显式 RPL 只做减法不做加法（除对侧列表复制），所以参考数与 reorder 数不会超过 SPS 宣告的 `max_dec_pic_buffering`（`GOPCfg.cpp:969-999`）。

### 2.3 单图尾段（残段仅 1 张，含整序列只有 1 张的极端情形）

- **参考关系**：
  - 若该图被提升为 intra（序列首图必然是 I 切片，`EncGOP.cpp:1667` 的 `CHECK(m_lastCodingNum == -1 && !m_isStartOfIntra, ...)` 强制编码从 I 开始），则 `numRefIdx[0]=numRefIdx[1]=0`（`EncGOP.cpp:2201-2202`），`constructRefPicList` 对 I 切片直接清零返回（`Slice.cpp:377-382`），完全自包含。
  - 若非 intra 的单张残段（理论上仅出现在已有前序 GOP 之后），两条 RPL 中所有指向残段外/未来的条目均被显式 RPL 重建剔除；若 L1 清空，B 切片在 `EncGOP.cpp:2211-2215` 被降级为 P 切片；若 L0 也为空则依赖 I 化路径。CuTree 测试中的 `"end of sequence"` 用例（`vvenc_unit_test.cpp:8497`）演示了单参考存活时 `ref1=MAX_INT`（无 L1）的形态。
- **输出顺序**：单张 AU 即全部输出。库测试 `test/vvenclibtest/vvenclibtest.cpp:521-539` 固定了这一流程：喂入 1 张图后循环以空缓冲调用 `vvenc_encode` 直到 `encodeDone=true`；`callingOrderNotRegular`（同文件 `556-604`）还验证了 flush 之后再喂图会被 API 拒绝（`vvencimpl.cpp:283` 的 `ERR_RESTART_REQUIRED`），即终止是单向不可逆状态。
- **NAL 形态**：首图按 `xGetNalUnitType` 与刷新类型决定为 IDR_N_LP/IDR_W_RADL（`EncGOP.cpp:1423-1448`），参数集 VPS/DCI/SPS/PPS 随第一个 IRAP 写出（`EncGOP.cpp:2565-2574`）。

### 2.4 lead/trail 图片的边界（分段编码场景）

分段模式（`VVENC_SEG_*`）会配置 lead/trail 图片（`vvencCfg.cpp:1474-1500`）。POC 超出 `framesToBeEncoded` 的 trail 图片在 `PicShared::reuse` 中被标记 `m_isTrail`（`EncStage.h:164-165`），仅 PreProcess（`processLeadTrail=true`，`EncLib.cpp:245`）与 MCTF（`EncLib.cpp:256`）处理它们，用于时域滤波；CuTree 与两级 GOP 编码器均以 `processLeadTrail=false` 初始化（`EncLib.cpp:266`、`278`、`287`），`addPicSorted` 直接跳过这类图片（`EncStage.h:308-316`），因此 **trail 图片永不进入 RPL、永不产生 AU**。`xGetStartOfLastGop` 也显式排除它们（`PreProcess.cpp:262-270`），保证尾部 GOP 起点判定只基于真实编码图片。

---

## 3. 源码证据索引与尾部状态必须保持的边界

### 3.1 关键证据对照表

| 机制 | 源码位置 |
|---|---|
| flush 标志在图片上的定义 | `source/Lib/CommonLib/Picture.h:236`，复位 `source/Lib/CommonLib/Picture.cpp:164,234` |
| API 终止状态机 | `source/Lib/vvenc/vvencimpl.cpp:265,283,359-361` |
| 阶段级 flush 传播与标记规则 | `source/Lib/EncoderLib/EncStage.h:305-376,379-442` |
| 顶层 flush 调度（nullptr IN / AU OUT） | `source/Lib/EncoderLib/EncLib.cpp:436-442,476-547` |
| 尾部 GOP 起点修正 | `source/Lib/EncoderLib/GOPCfg.cpp:269-280`；调用点 `source/Lib/EncoderLib/PreProcess.cpp:160-173` |
| 尾部 GOP 起点识别（排除 lead/trail） | `source/Lib/EncoderLib/PreProcess.cpp:259-295` |
| 模板 RPL 的可用性构造（availList/前向参考） | `source/Lib/EncoderLib/GOPCfg.cpp:388-502,1033-1075` |
| 裁剪 GOP 的"无下一 GOP"处理（bIsLast/e2） | `source/Lib/EncoderLib/GOPCfg.cpp:508-526`；裁剪本身 `671-713` |
| DPB 约束预计算与校验 | `GOPCfg.cpp:969-1031` |
| flush 下编码序连续性放宽 | `source/Lib/EncoderLib/EncGOP.cpp:1664`（另见 `1642-1645,1652`） |
| 默认 RPL 选择 | `EncGOP.cpp:2476-2493`；SPS RPL 初始化 `1341-1402` |
| 缺失参考检测 | `source/Lib/CommonLib/Slice.cpp:957-1053` |
| 显式 RPL 重建（剔除不可用条目） | `source/Lib/CommonLib/Slice.cpp:1055-1170+` |
| RPL 标记与 IDR 清空 | `Slice.cpp:561-619,858-954` |
| 参考就绪/可访问的启动门控 | `Slice.cpp:445-477`；`EncGOP.cpp:356-420,472-478` |
| RPL 到 Picture* 的实体解析 | `Slice.cpp:374-431` |
| CuTree 的 flush/MISSING 语义 | `source/Lib/EncoderLib/CuTree.cpp:558,617-619,657-719,760-780` |
| RC 对 flush 的响应 | `EncGOP.cpp:505-517,1711` |
| 终态资源释放 | `EncGOP.cpp:809-825` |
| AU 输出顺序与 DTS/CTS | `EncGOP.cpp:547-558,2495-2544`；重建 YUV 按 POC 序输出 `767-807` |
| 单图/flush 既有测试 | `test/vvenclibtest/vvenclibtest.cpp:521-539,556-604`；`test/vvenc_unit_test/vvenc_unit_test.cpp:8467-8517` |
| GOP-32 参考模板（既有材料） | `cfg/gop32.cfg:5-37` |

### 3.2 尾部状态必须保持的边界（代码中以 CHECK/条件强制，不可破坏）

1. **flush 标记只能落在尚未开始编码的图片上。** `EncStage.h:369` 的 `CHECK(!(*cpyItr)->isInitDone, ...)` 是硬边界；若图片已开始编码才被标记，`xInitPicsInCodingOrder` 中依赖 `picList.back()->isFlush` 的所有放宽（`EncGOP.cpp:1642,1652,1664`）都会失效或产生不一致 RPL。
2. **编码必须从 I 切片开始。** `EncGOP.cpp:1667` 的 CHECK 要求第一张图 `m_isStartOfIntra`；`fixStartOfLastGop` 对 GOP 0 非 POC0IDR 情形的 I 化（`GOPCfg.cpp:272-279`）是满足该不变量的尾部专属路径，不能跳过。
3. **RPL 中的每个 POC 必须能解析到真实 Picture*。** `constructRefPicList` 的循环不处理未命中情况（`Slice.cpp:400-408` 会迭代到 `end()` 再解引用），因此 `isRplPicMissing` + 显式 RPL 重建（`EncGOP.cpp:2182-2186`）必须先于它执行；IDR 路径靠 DPB 清空豁免（`Slice.cpp:959`）。
4. **参考只能沿编码序向后取，不能引用编码序上的未来图片。** 配置期由 `availList` 保证（`GOPCfg.cpp:501`），运行期由 `checkAllRefPicsReconstructed/Accessible` 与 `hasUnstartedRef` 保证（`EncGOP.cpp:401-405,417-418` 的 CHECKD）。CuTree 侧对应 `refCodingNum > ctx.codingNum => MISSING`（`CuTree.cpp:703-706`）。
5. **IDR/IRAP 的参考隔离。** IDR 之后的图片不能参考 IDR 之前的图片：`setDecodingRefreshMarking` 负责标记（`Slice.cpp:567-619`），`refPicIsFutureIDRnoLP` 防止把编码序上位于未来的 IDR 当参考（`Slice.cpp:1335-1343`，调用点 `1039,1085,1149`），CuTree 以 `refPoc < ctx.idr.poc/irap.poc => MISSING` 镜像同一规则（`CuTree.cpp:708-717`）。
6. **尾段不得突破 SPS 宣告的 DPB/reorder 上限。** 裁剪 GOP 列表在初始化时通过 `xCheckDBPConstraints`（`GOPCfg.cpp:1001-1031`）；显式 RPL 只剔除条目、按对侧列表补位，不引入新 POC，故尾部参考规模有界于完整 GOP。
7. **flush 是单向终态。** API 在 flushing 后拒绝新输入（`vvencimpl.cpp:283`），finalized 后拒绝一切编码调用（`vvencimpl.cpp:265`）；阶段内 `m_flush` 一旦置位不清除（`EncStage.h:399`）。这意味着尾部修正（I 化、RPL 裁剪）必须在首次 flush 调度时一次性完成，没有回退通道。
8. **lead/trail 图片不得进入编码域。** 阶段初始化的 `processLeadTrail` 差异（`EncLib.cpp:245,256` vs `266,278,287`）与 `addPicSorted` 的跳过逻辑（`EncStage.h:308-316`）共同保证尾部 GOP 判定与 RPL 构造只看见真实输出图片（`PreProcess.cpp:262-270`）。
9. **输出顺序严格跟随编码序队首。** `m_gopEncListOutput` 只从队首写出且要求 `isEncoded`（`EncGOP.cpp:547-556`），lockstep 模式还要求整个 chunk 完成（`548`）；尾段排空不能绕过该队列直接产出 AU，否则 DTS/CTS 关系（`2532-2543`）与 HRD 时序会被破坏。

---

## 4. 小结

vvenc 的尾部 GOP 处理不是单独的一套结构，而是**"同一套分层 GOP 机制 + flush 信号驱动的有界放宽"**：

- 结构上，GOP 模板在配置期已保证参考沿编码序可用，并通过裁剪列表与 `bIsLast` 假条目预置了"无下一 GOP"的形态；
- 信号上，`isFlush` 从 API 经 `EncStage` 逐阶段传播并固化到图片，解除队列深度与编码序连续性两处等待，同时触发 `fixStartOfLastGop` 把残段起点提升为 GOP/intra 边界；
- 参考安全上，I 化归零参考、`isRplPicMissing` 检测、显式 RPL 重建、RPL 标记与启动门控构成多层防线，确保任何指向不存在或过期图片的 RPL 条目在解析为 `Picture*` 之前被剔除；CuTree 在分析域内以 `NOT_ARRIVED→MISSING` 的 flush 语义镜像同一规则，并有单元测试固定；
- 三种尾段形态（完整 GOP / 短尾段 / 单图尾段）共享上述机制，区别只在 I 化是否触发、RPL 被剔除多少条目，而输出始终保持编码序、参考始终不越过编码序与 IRAP 隔离边界。
