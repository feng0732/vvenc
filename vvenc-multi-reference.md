# vvenc 多参考运动搜索对预测选择与率失真代价的影响

> 分析对象：本仓库 vvenc（VVC/H.266 编码器），分支 `task/45`，提交基线 `58cbc54`。
> 证据来源：仓库内 C++ 源码（行号基于当前工作树）、`cfg/` 预设文件与 `test/` 下的既有测试材料。
> 约定：每一节都明确区分 **【当前行为】**（源码可直接证实的实现）与 **【建议】**（分析得出的改进或验证方向，当前代码未实现）。

---

## 1. 总览：多参考搜索在编码流水线中的位置

### 1.1 调用链（当前行为）

一个非 I 帧的 CTU 按四叉树＋多类型树（QT/BT/TT）递归分区，每个叶子 CU 依次测试若干 `EncTestMode`，其中帧间相关的入口在 `EncCu::compressCtu` 的模式循环里（`source/Lib/EncoderLib/EncCu.cpp:1026-1058`）：

1. `ETM_MERGE_SKIP` → `EncCu::xCheckRDCostUnifiedMerge`（合并/Skip/MMVD/CIIP/GPM/仿射合并，`EncCu.cpp:1903`）
2. `ETM_INTER_ME` → `EncCu::xCheckRDCostInter`（AMVP 常规帧间，含 BCW 循环，`EncCu.cpp:3284`）→ `InterSearch::predInterSearch`（`InterSearch.cpp:1147`）
3. `ETM_INTER_IMV`（AMVR，受 `m_AMVRspeed` 控制）→ `EncCu::xCheckRDCostInterIMV`（`EncCu.cpp:3398`），内部同样调用 `predInterSearch`
4. 可选 IBC（屏幕内容）与各类 intra 模式

多参考（multi-reference）的核心循环在 `InterSearch::predInterSearch`：它对 **每个参考列表（L0/L1）的每个活跃参考索引** 各跑一次完整运动估计（ME），再用统一的率失真（RD）准则选出单向 L0、单向 L1 或双向预测。仿射模式在 `InterSearch::xPredAffineInterSearch`（`InterSearch.cpp:4836`）中重复同样的“列表 × 参考索引”双层循环结构。

### 1.2 两层代价体系（当前行为）

- **ME 阶段的运动代价**：`J_ME = SAD/SATD(失真) + motionLambda × 运动比特数`。`RdCost::setLambda` 中 `m_dLambdaMotionSAD = sqrt(m_dLambda)`（`source/Lib/CommonLib/RdCost.cpp:75-80`），运动比特通过 `RdCost::getCost(b)` 查表/相乘得到（`RdCost.h:197`）。
- **模式决策阶段的 RD 代价**：`J_RD = DistScale × SSE失真 + 比特数`，见 `RdCost::calcRdCost`（`RdCost.h:183-186`），其中 `DistScale = 2^SCALE_BITS / lambda`。
- 因此多参考引入的额外 refIdx/MVD 比特先在 ME 阶段以 sqrt(λ) 权重影响 MV 与参考索引选择，最终再在完整 RDO（含变换、量化、CABAC 比特）阶段被 λ 权重复核一次。参考越多，ME 调用次数线性增长，而每个 CU 的最终模式只保留一个代价最低者。

---

## 2. 参考画面列表（RPL）

### 2.1 数据结构与容量上限（当前行为）

| 常量 | 值 | 位置 | 含义 |
|---|---|---|---|
| `MAX_NUM_REF` | 16 | `CommonDef.h:209` | 语法层面单列表最大条目数 |
| `MAX_REF_PICS` | 6（默认构建；`INTER_FULL_SEARCH` 时 33） | `CommonDef.h:345-349` | 编码器内部按列表×索引保存 MV/AMVP 的数组维度，如 `cMvTemp[2][MAX_REF_PICS]`（`InterSearch.cpp:1158-1170`） |
| `VVENC_MAX_NUM_REF_PICS` | 29 | `include/vvenc/vvencCfg.h:80` | 配置/GOP 条目的 RPL 条目上限 |
| `AMVP_MAX_NUM_CANDS` | 2 | `CommonDef.h:225` | 每个（列表，refIdx）最终 MVP 候选数 |

### 2.2 RPL 的构造链路（当前行为）

1. **GOP 层定义**：每个 `GOPEntry` 带 `m_numRefPics[2]`、`m_numRefPicsActive[2]` 与 `m_deltaRefPics[2][...]`（相对当前 POC 的有符号偏移）。显式示例见 `cfg/gop32.cfg`：例如 Frame1（POC 32）L0 活跃 2 个、共 3 个条目（偏移 32/64/48），L1 活跃 2 个（偏移 32/64）；高层级 B 帧 L1 最多列 5 个条目（如 Frame6 的 `-1 -3 -7 -15 -31`），但活跃数普遍为 2。
2. **切片初始化**：`EncGOP::xInitFirstSlice` 先把 GOP 条目计数写入 `slice->numRefIdx`（`EncGOP.cpp:2173-2174`），再由 `xSelectReferencePictureList`（`EncGOP.cpp:2476-2493`）选择 SPS 中预建的 RPL 或本地 `initFromGopEntry`（`Slice.cpp:1928-1938`，按 delta POC 标识参考）。
3. **用户覆盖**：`NumRefPics` 配置经十进制编码按时间层裁剪活跃数——`numRefCode < 10` 时对所有层生效，否则逐位取模（`EncGOP.cpp:2196-2202`），并 `min(请求数, RPL 可用条目数)`。即**活跃参考数永远不会超过 GOP RPL 实际提供的条目数**。
4. **实体化**：`Slice::constructRefPicList`（`Slice.cpp:374-431`）按 POC 在 DPB 中匹配真实 `Picture*`，填入 `slice->refPicList[list][idx]`；`setRefPOCList`、`setList1IdxToList0Idx`（建立 L1→L0 同 POC 映射，`Slice.cpp:357-372`）、`setSMVDParam`（`Slice.cpp:276-355`）随后执行。
5. **退化处理**：B 切片若 L1 活跃数为 0，直接降级为 P 切片（`EncGOP.cpp:2211-2215`）；I 切片 RPL 清空（`Slice.cpp:377-382`）。

### 2.3 各预设的实际参考数（当前行为，来自预设源码与 cfg 文件）

| 预设 | `m_numRefPics`（L0/L1 相同语义） | 搜索范围 `m_SearchRange` | 双向 ME 范围 | 来源 |
|---|---|---|---|---|
| faster / fastest 类 | 1 | 128 | 1 | `vvencCfg.cpp:2715, 2772`；`cfg/randomaccess_faster.cfg:25` |
| fast | 222111（TL0/1 用 2，高层用 1） | 128 | 1 | `vvencCfg.cpp:2835`；`cfg/randomaccess_fast.cfg:25` |
| medium | 222221（仅最高 TL 用 1） | 384 | 4 | `vvencCfg.cpp:2905`；`cfg/randomaccess_medium.cfg:25` |
| slow / slower | 2 | 384 | 4 | `vvencCfg.cpp:2998, 3072`；`cfg/randomaccess_slow.cfg:25` |
| tooltest | 2 | — | — | `vvencCfg.cpp:3180` 附近 |

`NumRefPics=0` 表示使用 GOP 默认 RPL 全部活跃条目（低延迟实验预设均为 0，见 `cfg/experimental/lowdelay_*.cfg:37`）。

### 2.4 参考索引的信令代价（当前行为）

- ME 内部用截断一元码近似：列表条目数 >1 时 refIdx 代价为 `idx+1` 比特，最后一个索引减 1（`InterSearch.cpp:1238-1245` 单向、`1469-1476` 双向；仿射 `4920-4927` 相同）。条目数为 1 时 refIdx **零比特**。
- 实际比特流由 CABAC 上下文编码：`CABACWriter::ref_idx`（`CABACWriter.cpp:1810-1839`），`numRef <= 1` 直接不写比特；refIdx=0 仅 1 个 bin，越大索引 bin 越多。
- 后果：**refIdx=0 具有结构性的比特优势**。在 SAD 接近时，距离更远的高索引参考必须靠失真下降补偿其 1~N 个额外 bin，才能在 `J_ME`/`J_RD` 中胜出。

---

## 3. 候选生成：每个参考索引上发生了什么

### 3.1 单向预测循环（当前行为，`InterSearch.cpp:1232-1318`）

对每个列表、每个 `iRefIdxTemp ∈ [0, numRefIdx[list])`：

1. **AMVP 推导**：`xEstimateMvPredAMVP`（`InterSearch.cpp:1974-2032`）调用 `CU::fillMvpCand`（`UnitTools.cpp:1696-1805`），候选顺序为：
   - 左下 → 左（空间）；右上 → 上 → 左上（空间），去重后最多 2 个；
   - 不足 2 个且块宽高和 >12 时补 TMVP（同位块 C0/C1，受子图/CTU 边界条件限制，`UnitTools.cpp:1753-1788`）；
   - 再不足补 HMVP，最后用零 MV 填充到恰好 2 个。
   - 两个候选用模板代价 `xGetTemplateCost` 选优（严格 `>` 比较，`InterSearch.cpp:2011`），结果按 `(imv, list, refIdx)` 缓存在 `m_amvpResults` 中，BCW 多轮测试可直接复用（`InterSearch.cpp:1983-1992`）。
2. **整数像素 ME**：`xMotionEstimation`（`InterSearch.cpp:2165-2304`）。起始点包括 AMVP 与 `BlkUniMvInfoBuffer` 中邻近块的历史 uni-MV（`InterSearch.cpp:2237-2262`，新者优先）；随后按预设走全搜索或菱形/TZ 快速搜索（`xPatternSearchFast`，`InterSearch.cpp:2464-2504`）。
3. **分像素细化**：1/2、1/4 像素 DIF 模式搜索（`xPatternSearchFracDIF`，由 `m_fastSubPel` 控制是否跳过）。
4. **记账与选优**：代价加入 MVP 索引比特、refIdx 比特后，以**严格小于** `uiCost[list]` 才更新当前列表最优（`InterSearch.cpp:1298-1306`）。L0 的全部参考代价另存 `uiCostTempL0[]`，供低延迟快速路径复用。

### 3.2 双向预测循环（当前行为，`InterSearch.cpp:1346-1697`）

- 以两个列表各自的单向最优 MV/refIdx 为初值，交替细化（默认 4 次迭代；MODE2/MODE3 或 `mvdL1Zero` 时只迭代 1 次，`InterSearch.cpp:1414-1421`）。
- 每次迭代仍遍历当前列表的**全部** refIdx（`InterSearch.cpp:1463-1508`），用对侧预测生成的去除高频信号做 SAD（`removeHighFreq`，`InterSearch.cpp:2188`），双向 ME 使用独立的更小范围 `m_bipredSearchRange`（`InterSearch.cpp:2177`）。
- SMVD（对称 MVD）只在 `slice->biDirPred` 使能时测试，且固定使用 `symRefIdx[]` 指向的最近前/后向参考（`Slice.cpp:276-355`）；带 1.02 代价门限的提前退出（`InterSearch.cpp:1657`）。
- 低延迟 P 配置（`m_bFastMEForGenBLowDelayEnabled` 默认 true，`vvencCfg.cpp:526`）：L1 中与 L0 同 POC 的索引直接复用 L0 的 MV，只重算比特项，跳过整块 ME（`InterSearch.cpp:1260-1278`，依赖 `list1IdxToList0Idx`）。

### 3.3 合并类候选与多参考的关系（当前行为）

`CU::getInterMergeCandidates`（`UnitTools.cpp:909` 起）按 B1（上）、A1（左）、B0（右上）、A0（左下）、B2（左上）的顺序收集邻块运动信息，**每个候选自带其 refIdx**——即合并模式的“多参考”体现为继承邻居已经选定的参考索引，而不是自行搜索。MMVD 在合并候选基础上加 MVD 偏移；GPM 受块尺寸限制（minDim ≥ 8、maxDim ≤ 64，`EncCu.cpp:1977-1979`）；仿射合并要求 minDim ≥ 8（`EncCu.cpp:1967`）。合并候选先经 SATD 一轮剪枝（配额 `m_mergeRdCandQuota*`，小块用 `...SmallBlk` 配额，`EncCu.cpp:1956`），再进完整 RDO。

### 3.4 自适应搜索范围 ASR（当前行为）

`InterSearch::setSearchRange`（`InterSearch.cpp:6214-6238`）按 **(列表, refIdx)** 分别调范围：

```
SR = Clip3(minSearchWindow, m_SearchRange,
           (m_SearchRange * ADAPT_SR_SCALE * |curPOC-refPOC| + GOPSize/2) / GOPSize)
```

`ADAPT_SR_SCALE=1`（`CommonDef.h:232`）。时间距离越远的参考，搜索窗口越小（但不低于 `m_minSearchWindow`，faster/fast 为 96）；IRAP 帧禁用 ASR。默认 `m_bUseASR=false`（`vvencCfg.cpp:505`），各预设可打开。这意味着多参考中“远期参考”的 ME 更局部化，既省算力也降低抓到错误匹配的概率。

---

## 4. 块边界、块尺寸与编码模式的耦合

### 4.1 块尺寸对可选模式的硬约束（当前行为，均有源码常量支撑）

| 约束 | 阈值 | 位置 | 对多参考决策的影响 |
|---|---|---|---|
| 禁止双向预测 | 4×4，或宽+高=12（即 4×8/8×4） | `UnitTools.cpp:2224-2236`，调用点 `InterSearch.cpp:1347` | 小块只能在 L0/L1 单向中选，多参考列表减半 |
| BCW（双向加权）关闭 | 面积 < 256 | `CommonDef.h:420`；`EncCu.cpp:3295, 3436` | 小块 BCW 循环只跑 1 轮 |
| 仿射 AMVP 关闭 | 宽 ≤8 或高 ≤8 | `InterSearch.cpp:1782, 1822` | 小块不进入 `xPredAffineInterSearch` |
| TMVP 候选关闭 | 宽+高 ≤ 12 | `UnitTools.cpp:1753` | 极小块 AMVP 只剩空间候选＋零 MV |
| GPM 关闭 | minDim<8 或 maxDim>64 | `EncCu.cpp:1977-1979`；`CommonDef.h:472-475` | 几何合并不可用 |
| DMVR 关闭 | minDim<8 或面积<128，且要求常规合并双向等距 POC | `UnitTools.cpp:1304-1321` | 小合并块不做解码端 MV 细化 |
| CIIP 关闭 | 面积 <64 或 maxDim=CTU 尺寸 | `EncCu.cpp:1958` | 小块无合并/帧间混合模式 |
| 4×4 块不测试帧间模式 | 宽=高=4 | `EncCu.cpp:1026` | 4×4 直接跳过 MERGE/INTER_ME/IMV 入口 |

### 4.2 图像/CTU/子图边界对 MV 与候选的限制（当前行为）

- **MV 裁剪**：`xClipMvSearch`（`InterSearch.cpp:2306-2324`）把 MV 限制在图像边界外扩 8 像素、且不超过一个 CTU 尺寸的范围内；`xSetSearchRange`（`InterSearch.cpp:2355-2378`）对搜索窗两角分别裁剪。所有参考索引共用同一几何边界，但起始 MVP 不同，故实际可达位置随参考而变。
- **行级并行 IFP**：启用 `m_ifpLines` 时，垂直 MV 不得引用尚未重建的 CTU 行（`xCheckAndClipMvToFppLine`，`InterSearch.cpp:2337-2353`）；SMVD 与 `mvdL1Zero` 的 MVP 若越界会整体跳过该模式（`InterSearch.cpp:1369-1374, 1678-1680`）。
- **合并/AMVP 的可用性**：邻居查询经 `getCURestricted` 并受 `isDiffMER`（合并并行层级）、tile/slice/子图边界约束（`UnitTools.cpp:939, 972, 1008`）；TMVP 的 C0 位置还要满足子图右/下边界条件（`UnitTools.cpp:1767-1774`）。**位于图像边缘的块空间候选更少，零 MV 填充概率上升，多参考之间的 MVP 质量差异被放大**——好的参考索引若没有对应邻居 MVP，其 MVD 比特会升高。
- **分区隐式分裂**：达到 CTU/最小 QT 边界时 `getImplicitSplit` 强制四叉分裂（`EncModeCtrl.cpp:575-593`），边界处块尺寸被强制收小，间接触发上表的各类小块约束。

### 4.3 块尺寸对代价量级的影响（当前行为）

ME 代价中的 SAD 随像素数线性增长，而 refIdx/MVP/MVD 比特与块大小无关。因此**大块上失真项主导，更容易“买得起”高索引参考；小块上运动比特占比高，refIdx=0＋小 MVD 的候选结构性占优**。仿射门限 `affine_thr` 显式以 `log2(blk_area)` 为自变量（`InterSearch.cpp:1791-1820`），是同一原理的公式化体现。

---

## 5. 四类特殊情形分析

### 5.1 参考不足（参考条目少或列表退化）

**【当前行为】**

- 循环次数由 `slice->numRefIdx[list]` 直接决定（`InterSearch.cpp:1235`）。faster/fast 高层 TL 仅 1 个活跃参考：refIdx 零比特（`CABACWriter.cpp:1826-1829`），但完全失去备选画面，ME 只能在单一参考上找匹配。
- 序列起始：IDR/IRAP 后第一帧可引用的已编码画面少于 GOP RPL 标称条目时，`numRefIdx` 被 `min()` 截断（`EncGOP.cpp:2201-2202`）；RPL 条目缺失时走 `createExplicitReferencePictureSetFromReference` 补救（`EncGOP.cpp:2182-2185`）。
- B 切片 L1 无可用参考时整帧降级 P 切片（`EncGOP.cpp:2211`）；SMVD 找不到前/后向参考对时 `biDirPred=false`（`Slice.cpp:311-354`），SMVD 分支整体不执行。
- AMVP 候选不足时有完整兜底链（空间→TMVP→HMVP→零 MV，`UnitTools.cpp:1790-1805`），不会失败，但零 MVP 上的 MVD 可能很大。
- 快速退出：当 AMVP 常规帧间代价高于已测合并代价的 `MRG_FAST_RATIOMYV[TL]`（1.05~1.15，`CommonDef.h:453`）时，`predInterSearch` 直接返回 `true` 跳过残差编码（`InterSearch.cpp:1334-1345`）——参考越少、uni 代价越差，越容易触发该退出。

**【建议】**

- 可在单参考配置下记录“被零 MVP 填充的块比例”（通过新增 trace 计数），若比例高，说明 RPL 裁剪过激，可考虑对该时间层恢复第 2 个参考；当前代码没有这类反馈回路。
- 对参考不足帧可考虑放宽 ASR 下界（当前统一 `m_minSearchWindow`），用更大窗口弥补画面选择缺失；属配置策略层建议，无需改语法。

### 5.2 场景切换

**【当前行为】**

- 检测有两条路径：
  1. 预处理 STA（时空活动）检测：`PreProcess::xDetectSTA`（`PreProcess.cpp:436-471`），活动度比值超阈值时把 GOP 条目切片类型改为 `'I'`、标记 `SCT_TL0_SCENE_CUT`，并可按 `m_sliceTypeAdapt` 重启 intra 周期。
  2. 两遍码控的 `RateCtrl::detectSceneCuts`（`RateCtrl.cpp:877-923`）：基于视觉活动度跳变与 TL0/1 PSNR 落差 >4.5dB 判新场景，且两次切换最小间隔为 `(GOPSize+1)/2`，过近的切换被过滤。
- 场景切换帧成为 adapted I 帧（intra），**其后的帧 RPL 仍按 GOP 结构构造，但跨场景的旧画面残留在 RPL 前部时，多参考 ME 会给它们算出极高 SAD**，在严格 `<` 比较下自然落选，代价是浪费了对应 ME 次数（每个 refIdx 一次完整 ME）。
- SCC 强场景（`isSccStrong`）切换 `useNumRefs=true`（`Picture.cpp:420`），改用独立的 `m_numRefPicsSCC` 计数（`EncGOP.cpp:2196`），预设中该值多为 0（跟随默认）或 1，并同时切换 SCC 专用 ME 方法（`InterSearch.cpp:2471-2486`）。
- 场景切换 GOP 还触发 QP 调整标记 `isSceneCutGOP/isSceneCutCheckAdjQP`（`EncGOP.cpp:2083-2109`），改变 λ，进而同时改变运动代价权重与最终 RD 判据。

**【建议】**

- 当前实现没有“场景切换后临时缩短 RPL”的机制：旧场景参考仍被逐一搜索。可在检测到 `SCT_TL0_SCENE_CUT` 后的 1~2 个 GOP 内，把活跃参考数临时下调到 1（复用现有 `NumRefPics` 时间层编码通道即可实现），用少量 BD-rate 风险换 ME 耗时下降；建议先以 trace 量化这些参考的实际命中率再决定。
- `detectSceneCuts` 的 4.5dB PSNR 阈值与最小间隔是硬编码（`RateCtrl.cpp:879, 895`），对低帧率/高运动素材可能误过滤，建议暴露为隐藏配置项便于调优。

### 5.3 不同块尺寸

**【当前行为】**（汇总第 4 节，并补充决策侧证据）

- 小块（4×4/4×8/8×4）：无双预测、无仿射、无 GPM/DMVR/CIIP，AMVP 无 TMVP；多参考实际退化为“单列表单参考＋空间 MVP”的选择。
- 中块（8×8 ~ 16×16）：全部帧间工具开放，但 BCW 在面积 <256 时关闭；合并 SATD 配额走 `m_mergeRdCandQuotaRegularSmallBlk`（`EncCu.cpp:1956`），候选审查更激进。
- 大块（≥64×64 量级）：失真主导，高 refIdx 的额外 bin 相对廉价；仿射 MLR 门限随面积放宽（`InterSearch.cpp:1791-1820`），但 `m_Affine>2` 时 TLayer>3 直接关闭仿射（`InterSearch.cpp:1768-1781`），时间层约束与尺寸约束叠加。
- 模式统计输出：`ENABLE_CU_MODE_COUNTERS` 构建下，编码器结束时按片型/时间层与块形状两维打印实测模式计数（`EncLib.cpp:158-172`），可直接观察不同尺寸上 inter/skip/intra 的分布。

**【建议】**

- 可在该统计中增加 refIdx 直方图（当前计数器只到模式类别，见 `Rom.h` 的 `CU_MODES_TESTED`），用于验证“大块更多使用高索引参考”这一推断；目前只能通过 D_ME trace 间接观察。

### 5.4 代价相同（tie）的处理

**【当前行为】** 全链路一致使用**严格小于**，等值时保留先评估的候选，评估顺序因此成为隐式 tie-break：

| 决策点 | 比较 | 位置 | 等值时的赢家 |
|---|---|---|---|
| TZ 搜索点接受 | `cost < uiBestSad` | `InterSearch.cpp:518`（`xTZAcceptIfBetter`） | 先搜到的点（菱形/方格遍历序） |
| AMVP 模板候选 | `uiBestCost > uiTmpCost` | `InterSearch.cpp:2011` | 索引 0（空间左/上优先于 TMVP/HMVP/零 MV） |
| 单列表参考选优 | `uiCostTemp < uiCost[list]` | `InterSearch.cpp:1298` | **refIdx 更小者**（循环从 0 递增） |
| 双向参考选优 | `uiCostTemp < uiCostBi` | `InterSearch.cpp:1486` | 先迭代的列表/低索引 |
| SMVD 与常规双向 | `symCost < uiCostBi` | `InterSearch.cpp:1677` | 常规双向 |
| 仿射 vs 常规 | `uiHevcCost <= uiAffineCost` | `InterSearch.cpp:1925` | 常规（此处为 ≤，仿射仅严格更优才替换） |
| 4 参数 vs 6 参数仿射 | `uiAffineCost <= uiAffine6Cost` | `InterSearch.cpp:1896` | 4 参数 |
| CU 模式总决策 | `tempCS->cost < bestCS->cost` | `EncModeCtrl.cpp:1142` | 先测试的模式（merge/skip 先于 INTER_ME，见 `EncCu.cpp:1029-1042`） |

后果：在失真与比特完全同值时，输出确定性地偏向 **低 refIdx、低 MVP 索引、先测试的简单模式**。这与信令设计方向一致（低索引本来 bin 更少，真正完全等值的概率低），且保证了单线程/多线程下结果可重复。

**【建议】**

- 保持严格 `<` 语义（确定性是优点），但建议在代码注释或文档中显式写明“评估顺序即 tie-break 顺序”——目前该性质只能靠通读比较符推断。
- 若未来引入并行评估多参考（如按 refIdx 并发 ME），必须在合并结果时恢复 refIdx 升序的确定性归并，否则等值场景会产生线程相关的非确定性输出；当前串行实现无此问题。

---

## 6. 率失真代价如何被多参考改变：机制总结

1. **比特侧**：每增加一个活跃参考，refIdx 语法的截断一元/CABAC 平均码长增加（`CABACWriter.cpp:1810`）；被选中的高索引参考支付额外 bin，未被选中的参考不产生码流比特但消耗 ME 算力。MVP 索引比特（`m_auiMVPIdxCost`，`InterSearch.cpp:1258`）与 MVD 比特随每个（列表，参考）组合独立计算。
2. **失真侧**：每个参考独立做整数＋分像素 ME，给出该参考下可达的最小 SAD；多参考等于扩大了运动补偿的假设空间，遮挡/重复纹理/非线性运动区域可找到更匹配画面。
3. **λ 耦合**：ME 用 sqrt(λ)（`RdCost.cpp:79`）使运动搜索对码长较不敏感、侧重匹配质量；完整 RDO 用 λ 复核。高 QP（λ 大）时运动比特权重上升，refIdx=0 与合并模式更受青睐；低 QP 时高索引参考更易凭失真胜出。
4. **快速算法的交互**：ASR 按参考时间距离缩窗（`InterSearch.cpp:6234`）、双向独立小窗（`m_bipredSearchRange`）、低延迟同 POC 复用 L0 MV（`InterSearch.cpp:1260-1278`）、BCW 缓存 uni-motion 免重搜（`xReadBufferedUniMv`，`InterSearch.cpp:4658-4680`）、合并 SATD 剪枝（`EncCu.cpp:2016-2026`）共同决定了“多参考”在快速预设下实际搜索到什么程度。
5. **模式门限联动**：`FastInferMerge` 的 1.05~1.15 倍门限（`InterSearch.cpp:1339`）、仿射 1.05 倍门限（`InterSearch.cpp:1858`）、AMVR 的 1.06/1.25 倍停止代价（`EncCu.cpp:3481`）都是以当前最优代价为基准的相对判决——多参考改善了基准代价后，会连带压制仿射/AMVR/残差模式的测试机会。

---

## 7. 可观察的编码决策（验证手段清单）

以下手段全部基于仓库已有设施，无需改语法：

| 观察目标 | 设施 | 入口/证据 |
|---|---|---|
| 每次 ME 的（列表，refIdx，块位置/尺寸，SAD 代价，MV） | DTRACE `D_ME` 通道 | trace 点 `InterSearch.cpp:2281, 2303`；通道定义 `dtrace_next.h:136`；CLI `--tracefile/--trace`（`VVEncAppCfg.h:1197`），需 ENABLE_TRACING 构建 |
| 最终选中的 interDir/refIdx/MVD/merge 等语法 | DTRACE `D_SYNTAX` 与 CABAC writer trace | 如 `CABACWriter.cpp:1806, 1834` |
| 各块形状/片型/时间层上的模式分布（inter、skip、affine 等） | CU 模式计数器 | `EncLib.cpp:158-172`，需 `ENABLE_CU_MODE_COUNTERS` 构建，stdout 直接输出 |
| 帧级 P/B 计数、PSNR、码率 | Analyze 摘要 | `EncGOP.cpp:873-882`（全部/I/P/B 分文件摘要） |
| RPL 实际条目与活跃数 | 配置确认＋日志 | 构造链路 `EncGOP.cpp:2173-2203`；`NumRefPics` 十进制语义见各 `cfg/randomaccess_*.cfg:25` 注释 |
| 端到端冒烟验证 | 既有测试素材 `test/data/RTn23_80x44p15_f15.yuv`（80×44、15fps、15 帧）＋ `RTn23.cfg`（编 5 帧、IntraPeriod=32） | `test/data/RTn23.cfg`；库测试 `test/vvenclibtest/vvenclibtest.cpp:168-172`（GOPSize 16/32 遍历） |
| 预设参数核对 | 预设表 | `vvencCfg.cpp:2679-3180`（faster→tooltest） |

建议的具体实验矩阵（在上述设施上执行）：

1. 同一 YUV 分别用 `NumRefPics=1` 与 `NumRefPics=2`（medium 预设）编码，用 D_ME trace 统计每个 refIdx 的**被选率**与其 SAD 分布，量化多参考的真实增益来源。
2. 构造含硬切的序列（或在 RTn23 后拼接异质内容），对照切换帧前后 refIdx 命中率与 I 帧位置，验证 5.2 节的落选浪费推断。
3. 用 CU 模式计数器比较 80×44 小分辨率（大量 4×4/8×8 块）与 1080p 序列的 interDir 分布，验证小块双向禁用（`UnitTools.cpp:2224`）的实际覆盖面。
4. 固定 QP 扫描，统计输出码流中 refIdx bin 的实际占比（D_SYNTAX），核对 2.4 节的比特结构性偏置。

> 说明：本环境未提供预编译产物，上述实验为**可执行的验证方案**；本报告的行为结论本身均已由所引源码行直接证实，不依赖实验结果。

---

## 8. 结论

**【当前行为，源码证实】**

1. vvenc 的多参考是“RPL 裁剪 → 逐（列表，refIdx）AMVP＋ME → 严格 `<` 选优 → 单向/双向/SMVD/仿射统一 RD 竞赛”的确定性流水线；活跃参考数受 GOP RPL、`NumRefPics` 时间层编码与 DPB 可用画面三重约束。
2. 参考索引带有结构性码长梯度（refIdx=0 最短，单参考零比特），叠加全链路严格小于比较，使 tie 场景确定性偏向低索引、简单、先测试的模式。
3. 块尺寸通过一组硬阈值（4×4 禁双向、256 面积禁 BCW、8 维限仿射/GPM/DMVR/TMVP）改变多参考可达的模式空间，边界块还因邻居候选受限而推高 MVD 比特。
4. 场景切换靠 STA 与两遍码控两条路径检测并转 I 帧，但旧场景参考不会被主动摘除，仍消耗全额 ME 次数；参考不足时有切片降级、SMVD 关闭、零 MV 填充与合并快速退出等完整兜底。
5. 快速预设把参考数压到 1~2 并配合 ASR、小双向窗、低延迟 MV 复用与合并剪枝，使“多参考”的算力随预设档位显著伸缩。

**【建议，尚未实现】**

1. 场景切换后临时下调活跃参考数，并先用 D_ME trace 量化旧参考命中率作为决策依据（5.2）。
2. 在 CU 模式计数器中增加 refIdx/MVP 索引直方图，把当前只能从 trace 推断的参考选择行为变为一等统计输出（5.3）。
3. 把 `detectSceneCuts` 的硬编码阈值参数化（5.2）；单参考帧考虑放宽 ASR 下界以补偿画面选择缺失（5.1）。
4. 在比较点注释中固化“评估顺序即 tie-break”的约定，并在任何未来并行化 ME 的归并阶段保持该顺序（5.4）。

---

### 附：关键源码索引

| 主题 | 文件:行 |
|---|---|
| 多参考主循环与最终 interDir 决策 | `source/Lib/EncoderLib/InterSearch.cpp:1147-1971`（选优 1298、1486、1722-1761） |
| 单参考 ME（ASR 取窗、起始点、分像素） | `InterSearch.cpp:2165-2304` |
| 自适应搜索范围 | `InterSearch.cpp:6214-6238` |
| AMVP 候选填充与模板选优 | `UnitTools.cpp:1696-1805`；`InterSearch.cpp:1974-2032` |
| 合并候选生成与 SATD 剪枝 | `UnitTools.cpp:909+`；`EncCu.cpp:1903-2026` |
| RPL 构造与活跃数裁剪 | `EncGOP.cpp:2173-2215, 2476-2493`；`Slice.cpp:276-431, 1928-1938` |
| refIdx CABAC 信令 | `CABACWriter.cpp:1810-1839` |
| 小块/工具尺寸约束 | `UnitTools.cpp:2224-2236, 1304-1321, 1753`；`CommonDef.h:420, 472-475`；`EncCu.cpp:1958, 1967, 1977` |
| MV 边界裁剪与 IFP 行限制 | `InterSearch.cpp:2306-2378` |
| 场景切换检测 | `PreProcess.cpp:436-471`；`RateCtrl.cpp:877-923`；`EncGOP.cpp:2083-2109` |
| RD/运动 λ 与代价函数 | `RdCost.cpp:75-80`；`RdCost.h:183-218` |
| 模式级严格小于判决 | `EncModeCtrl.cpp:1142` |
| 预设参考数/搜索范围 | `source/Lib/vvenc/vvencCfg.cpp:2679-3180`；`cfg/randomaccess_*.cfg:25-26` |
| GOP RPL 实例 | `cfg/gop32.cfg` |
| 测试素材 | `test/data/RTn23_80x44p15_f15.yuv`、`test/data/RTn23.cfg`、`test/vvenclibtest/vvenclibtest.cpp` |
