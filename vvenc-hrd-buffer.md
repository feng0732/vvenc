# VVenC 码率缓冲 HRD 时序源码分析报告

分析对象：本仓库（分支 `task/31`，基线提交 `58cbc54`）中的 VVenC 编码器。
范围：HRD（Hypothetical Reference Decoder）信令的生成、它与码率控制 / 画面重排序 / 缓冲状态的关联，以及编码器与封装（应用）层之间的责任划分。

---

## 1. 总览：HRD 在 VVenC 中的位置

VVenC 的 HRD 实现由三块组成：

| 模块 | 文件 | 职责 |
|---|---|---|
| 参数模型 | `source/Lib/CommonLib/HRD.h` | `GeneralHrdParams` / `OlsHrdParams` / `HRD` 数据结构，镜像 VVC 规范的 `hrd_parameters()` 语法 |
| 参数计算 | `source/Lib/EncoderLib/EncHRD.cpp` | `EncHRD::initHRDParameters()` 由配置和 SPS 推导 HRD 参数 |
| SEI 生成 | `source/Lib/EncoderLib/SEIEncoder.cpp` | `initBufferingPeriodSEI()` / `initPictureTimingSEI()` 生成缓冲期与图像定时 SEI |

关键事实：**VVenC 不维护一个逐比特的 CPB（Coded Picture Buffer）占用模型**。`SEIEncoder.cpp:192-196` 的注释明确写道：

> "A commercial encoder should track the buffer state for all layers and sub-layers to ensure CPB conformance. Such tracking is required for calculating alternative CPB parameters. Unfortunately VTM does not have such tracking."

因此 `altCpbParamsPresent` 被硬编码为 `false`（`SEIEncoder.cpp:197-198`）。HRD 信令是"声明式"的：编码器声明一组 HRD 参数和定时 SEI，但不模拟解码器缓冲的充盈/抽空过程。码率符合性由码率控制（RC）在 GOP 粒度上近似保证（见 §2.2）。

## 2. HRD 信令与码率控制的关联

### 2.1 HRD 参数直接取自 RC 配置

`EncHRD::initHRDParameters()`（`EncHRD.cpp:67-160`）在编码器初始化时被调用一次（调用点 `EncGOP.cpp:193`，经 `xInitHrdParameters()`，`EncGOP.cpp:1404-1415`），并把结果拷入 SPS：

- **码率**：`bitRate = encCfg.m_RCTargetBitrate`（`EncHRD.cpp:77`）。若 RC 关闭（`m_RCTargetBitrate == 0`），`rateCnt` 为 false，`generalNalHrdParamsPresent` / `generalVclHrdParamsPresent` 均为 false（`EncHRD.cpp:85-88`）——即 HRD 语法结构仍在 SPS 中（`sps.hrdParametersPresent` 默认 true，`vvencCfg.cpp:612`），但不含任何码率/CPB 子层参数。
- **CPB 大小**：`cpbSize = profileLevelTierFeatures.getCpbSizeInBits()`（`EncHRD.cpp:78`），取自所选 Level 的最大 CPB，并 `CHECK` 非零（`EncHRD.cpp:80`）。注释说明"using bitRate results in 1 second CPB size"（`EncHRD.cpp:138`）——实际用的是 Level 上限而非 1 秒码率。
- **时间尺度**：`timeScale = m_FrameRate`，`numUnitsInTick = m_FrameScale`（`EncHRD.cpp:82-83`），即 HRD 时钟与帧率绑定。
- **缩放**：`xCalcScale()`（`EncHRD.cpp:49-65`）计算 `bitRateScale` / `cpbSizeScale`，使 `bit_rate_value_minus1` / `cpb_size_value_minus1` 落入可表示范围；`hrdCpbCntMinus1 = 0`（单 CPB 规格），`cbrFlag = false`（声明为 VBR，`EncHRD.cpp:151/156`）。

SPS 侧写出在 `VLCWriter.cpp:979-991`：`sps_timing_hrd_params_present_flag` 后跟 `codeGeneralHrdparameters()` 与 `codeOlsHrdParameters()`。

### 2.2 配置层把 HRD SEI 与 RC 强绑定

`vvencCfg.cpp:786-790`：当 `m_RCTargetBitrate == VVENC_RC_OFF` 时，开启 `m_bufferingPeriodSEIEnabled` 或 `m_pictureTimingSEIEnabled` 直接报参数错误（"Enabling bufferingPeriod SEI requires rate control"）。`vvencCfg.cpp:2256-2257` 进一步要求这两个 SEI 依赖 `m_hrdParametersPresent`。也就是说：**HRD 定时 SEI 只在 RC 开启时才允许出现**。

### 2.3 RC 用 GOP 级封顶代替 CPB 模拟

`RateCtrl.cpp:96`：

```cpp
maxGopRate = int( 0.5 + std::min( ..., std::min( 3 * targetRate, maxBitrate ) * GOPSize / frRate ) );
```

即每个 GOP 的比特预算被限制在 `min(3×目标码率, RCMaxBitrate)` 对应的 GOP 比特数内。`RateCtrl.cpp:978-1030` 的 "pre-capper" 在 lookahead 关闭时把超预算 GOP 的比特按 `gopTempVal` 比例压回，并把节省的比特（`savedBits`）再分配给后续 GOP；`RateCtrl.cpp:1236-1269` 在逐帧目标比特分配时再次以 `maxGopRate` 截顶。这是一种**防止 CPB 上溢的保守近似**（限制短期峰值），但它不跟踪缓冲充盈度，因此**不防止、也不检测 CPB 下溢**（缓冲抽空）。

## 3. 画面重排序与缓冲状态

### 3.1 重排序参数的来源

- 开关：`m_picReordering`（默认 1，`vvencCfg.cpp:386`；命令行 `--PicReordering`，`VVEncAppCfg.h:718`）。关闭时（低延迟场景）`m_maxTLayer = 0`（`EncCfg.cpp:91`）。
- GOP 结构推导：`GOPCfg.cpp:976-995` 由 GOP 列表计算每层 `m_numReorderPics[tid]` 与 `m_maxDecPicBuffering[tid] = numReorderPics + 1`。
- 写入 SPS：`EncGOP.cpp:1148-1149` 把 `maxDecPicBuffering` / `numReorderPics` 写入 SPS 的 PTL/DPB 参数。

### 3.2 重排序如何进入 HRD 定时 SEI

**(a) Buffering Period SEI**（`SEIEncoder.cpp:70-199`，在每个 IRAP/GDR 的 AU 上由 `EncGOP.cpp:2782-2793` 触发）：

- `isRandomAccess = m_picReordering`（`SEIEncoder.cpp:101`）。随机访问且 `IntraPeriod < 256` 时，`cpbRemovalDelayLength = dpbOutputDelayLength = ceilLog2(IntraPeriod)+1`；否则（低延迟或长帧内周期）固定为 9 bit（`SEIEncoder.cpp:102-111`）。
- `initialCpbRemovalDelay` 固定为 45000（90 kHz 时钟下 0.5 秒，`SEIEncoder.cpp:74`）。
- `cpbRemovalDelayDelta` 表只支持 CTC 风格的 GOP 8 / GOP 16 两种结构（`SEIEncoder.cpp:119-177`），其他 GOP 尺寸直接 `THROW`。
- `dpbOutputTidOffset[i]` 由 `numReorderPics` 逐层计算（`SEIEncoder.cpp:178-191`），把子层重排序深度折算进 DPB 输出偏移。

**(b) Picture Timing SEI**（`SEIEncoder.cpp:236-`，每帧由 `EncGOP.cpp:2802-2808` 触发，且要求 `rcIsFinalPass && !m_isPreAnalysis`）：

- `auCpbRemovalDelay[最高子层] = m_totalCoded - m_lastBPSEI`（自上一个缓冲期 SEI 以来编码的 AU 数），并被 clamp 到 `[1, 2^cpbRemovalDelayLength]`，超界触发 `CHECK`（`SEIEncoder.cpp:263-264`）。
- 低子层的延迟按 GOP 内位置查 `cpbRemovalDelayDeltaIdx` 表（`SEIEncoder.cpp:266-423`），或按 `scaledDistToBuffPeriod` 缩放（`SEIEncoder.cpp:426-428`）。
- **DPB 输出延迟直接体现重排序**（`SEIEncoder.cpp:431`）：

  ```cpp
  ptSei->picDpbOutputDelay = sps->numReorderPics[maxTLayers-1] + slice->poc - m_totalCoded[maxNumSubLayers-1];
  ```

  即"显示序 POC 与解码序序号之差 + 最大重排序深度"。被注释掉的 `EncGOP` 高效场编码修正（`SEIEncoder.cpp:432-436`）说明该近似在图像交换场景下可能差 1。

### 3.3 输出时间戳（cts/dts）中的重排序延迟

库输出 AU 时按**解码序**写出，时间戳在 `EncGOP.cpp:2530-2543` 生成：

```cpp
const int64_t iDiffFrames = m_numPicsCoded - pic.poc - pic.picsInMissing;
au.dts = ( ( iDiffFrames - m_maxTLayer + m_numPicsOutOffset ) * m_ticksPerFrameMul4 ) / 4 + au.cts;
```

- `cts` 由应用层透传（见 §4），`dts` = cts 减去一个以 `m_maxTLayer`（= ceilLog2(GOPSize)，重排序开启时）近似的重排序延迟，再补上丢帧偏移 `m_numPicsOutOffset`。
- 丢帧检测在 `EncGOP.cpp:253-271`：相邻输入帧 `cts` 差超过 2 帧或为负时，把差值折算成 `picOutOffset` 累积进 `m_numPicsInMissing`，使后续 dts 保持单调。
- 注意 `m_maxTLayer` 只是重排序延迟的**近似**（GOP 8 → 3，GOP 16 → 4），与 `numReorderPics` 的真实值可能不同。

## 4. 编码器与封装层的责任划分

| 责任 | 编码器库（Lib） | 应用/封装层（App） |
|---|---|---|
| HRD 参数语法（SPS `hrd_parameters()`） | 生成（`EncHRD` + `VLCWriter.cpp:979-991`） | 无 |
| Buffering Period / Picture Timing SEI | 生成并嵌入 AU（`EncGOP::xWriteLeadingSEIs`，`EncGOP.cpp:2775-2808`；写出 `SEIwrite.cpp:275-`） | 无 |
| AU 输出顺序 | 按解码序输出（`EncGOP.cpp` 写 NAL 单元） | 原样写入 Annex B 流 |
| cts 生成 | 透传（`au.cts = pic.cts`，`EncGOP.cpp:2531`） | `vvencapp.cpp:390-393` 按 `m_TicksPerSecond`（默认 27000000，`vvencCfg.cpp:359`）由帧序号生成 |
| dts 生成 | 由 cts 和重排序近似延迟计算（`EncGOP.cpp:2540-2542`） | 仅消费 |
| CPB 符合性（不溢出/不抽空） | 仅 GOP 级码率封顶近似（§2.3），无缓冲占用跟踪 | **完全在外部**：需用 HRD 验证器/解码器检验 |
| 容器层交织（MP4 等 stts/ctts） | 不涉及 | 由外部封装工具根据 cts/dts 处理 |

`vvencFFapp`（`source/App/vvencFFapp/EncApp.cpp:255-`）同样只把 AU 写出，不做任何 HRD 级处理。库 API 默认 `cts/dts` 无效（`vvenc.cpp:185-188`），由调用方决定是否提供输入 cts。

## 5. 三种边界情形的可观察行为

### 5.1 缓冲接近上下界

**上界（CPB 溢出风险）**：
- 编码器侧唯一的防护是 RC 的 GOP 级封顶（`RateCtrl.cpp:96, 978-1030, 1236-1269`），把单 GOP 比特限制在 `min(3×target, RCMaxBitrate)` 对应的预算内。可观察结果：高动态段的 QP 被抬高、GOP 实际比特被压回预算；输出流的短期峰值码率受限。
- 没有逐 AU 的 CPB 占用模拟，因此**无法从编码器输出中观察到"缓冲还剩多少"**；`altCpbParamsPresent = false`（`SEIEncoder.cpp:197`）也意味着流中没有备选 CPB 参数可供解码器切换。

**下界（CPB 抽空/underflow）**：
- 无任何防护或检测。低复杂度场景下实际码率远低于 `bit_rate_value` 时，声明的 `au_cpb_removal_delay` 仍按固定节奏递增（`auCpbRemovalDelayDelta = 1`，`SEIEncoder.cpp:117`），编码器不会调整初始延迟或插入填充。可观察结果：流本身语法合法，但是否满足 HRD 下溢约束只能由外部验证器判定。

**计数溢出防护（语法层面）**：
- `auCpbRemovalDelay` 被 clamp 到 `2^cpbRemovalDelayLength` 并带 `CHECK`（`SEIEncoder.cpp:263-264, 427-428`）；低延迟/长帧内周期时长度只有 9 bit（`SEIEncoder.cpp:109-110`），若两个缓冲期 SEI 之间编码 AU 数超过 512，编码器会**直接断言失败**（`CHECK ... "cpbRemovalDelayLegth too small"`），而不是静默回绕。可观察结果：长 GOP 间距离开启 PictureTiming SEI 时编码崩溃，这是最明显的"缓冲参数触界"信号。

### 5.2 参数集变化

- **运行中改配置不支持**：`VVEncImpl::reconfig()` 直接返回 `VVENC_ERR_NOT_SUPPORTED`（`vvencimpl.cpp:113-117`）。因此不存在"码率改变 → HRD 参数更新"的路径。
- HRD 参数只在编码器初始化时计算一次（`EncGOP.cpp:193`），之后即使码率统计变化，SPS 中的 `bit_rate_value` / `cpb_size_value` 不变。
- `m_rewriteParamSets` 开启时，每个 IRAP 重发 VPS/SPS/PPS（`EncGOP.cpp:2565`），但内容相同——可观察结果是参数集重复出现，HRD 参数不变。
- **缓冲期 SEI 的重置**：每个 IRAP/GDR 都重新生成 Buffering Period SEI（`EncGOP.cpp:2782-2793`），并把 `m_lastBPSEI[i] = m_totalCoded[i]`（`SEIEncoder.cpp:443-448`），即每个 IRAP 开启一个新的 HRD 缓冲期，`auCpbRemovalDelay` 重新从 1 计数。IDR/CRA 还会置 `m_rapWithLeading = true`（`SEIEncoder.cpp:449-452`），切换 `cpbRemovalDelayDeltaIdx` 查表分支。可观察结果：每个 IRAP 处 initialCpbRemovalDelay 重新声明为 0.5 s，PT SEI 的 removal delay 计数重启。

### 5.3 重排序延迟

- 开启重排序（默认）：`m_maxTLayer = ceilLog2(GOPSize)`，SPS 声明 `numReorderPics`（GOPCfg 推导），PT SEI 的 `picDpbOutputDelay = numReorderPics + poc - 解码序号`（`SEIEncoder.cpp:431`）。可观察结果：输出 AU 的 dts 比 cts 提前约 `maxTLayer` 帧（`EncGOP.cpp:2540`），前若干帧 dts 可能为负；解码器按 PT SEI 延迟输出，显示序恢复为 POC 序。
- 关闭重排序（`--PicReordering 0`，低延迟）：`m_maxTLayer = 0`，`numReorderPics = 0`，`picDpbOutputDelay = poc - 解码序号 = 0`（稳态），dts ≈ cts；BP SEI 的延迟字段长度退化为 9 bit（`SEIEncoder.cpp:108-111`）。可观察结果：端到端延迟最小，但 HRD 字段的动态范围变小（见 §5.1 的 9 bit 溢出风险）。
- 丢帧/变帧率输入：`picOutOffset` 机制（`EncGOP.cpp:253-271`）把 cts 空洞折算进 dts，保持 dts 单调且与真实输入节奏对齐；但 HRD SEI 的 `auCpbRemovalDelay` 仍按"已编码 AU 数"计数，不感知输入丢帧——两者在丢帧时会出现语义偏差。

## 6. 已有测试材料

- **参数合法性测试**：`test/vvenclibtest/vvenclibtest.cpp:370-372`
  - `useHrdParametersPresent`：验证 `m_hrdParametersPresent` 取 0/1 均可初始化；
  - `useBufferingPeriodSEIEnabled` / `usePictureTimingSEIEnabled`：在 `m_RCTargetBitrate = 0`（RC 关闭）前提下开启这两个 SEI，期望**初始化失败**（`expectedFail = true`），即回归测试 §2.2 的"HRD SEI 需要 RC"约束。
- **冒烟配置**：`test/data/RTn23.cfg`（80×44、15 fps、5 帧、`IntraPeriod=32`、`PerceptQPA=1`）+ `test/data/RTn23_80x44p15_f15.yuv`，是 vvenclibtest 实际编码所用的最小序列；该配置未开启 RC 与 HRD SEI，走的是"HRD 语法在 SPS 中但无码率子层参数"的路径（§2.1）。
- 仓库中没有针对 CPB 占用、HRD 符合性或 dts/cts 单调性的单元测试（`test/vvenc_unit_test` 不含 HRD 用例）；HRD 行为目前只有上述参数级回归覆盖。

## 7. 当前行为（事实陈述）

1. HRD 参数在初始化时由 `m_RCTargetBitrate` 与 Level 上限 CPB 推导一次，写入 SPS；`hrdCpbCntMinus1 = 0`、`cbrFlag = false`（恒声明单 CPB、VBR）。
2. Buffering Period SEI 在每个 IRAP/GDR 重新生成，初始 CPB 移除延迟恒为 0.5 s（90 kHz）；Picture Timing SEI 每帧生成，`auCpbRemovalDelay` 按 AU 计数递增，超界即 `CHECK` 崩溃。
3. 重排序延迟以 `numReorderPics` 进入 PT SEI 的 `picDpbOutputDelay`，以 `m_maxTLayer` 近似进入输出 dts；两处近似来源不同，可能不一致。
4. 码率符合性仅靠 GOP 级 `maxGopRate` 封顶近似，无 CPB 占用跟踪、无下溢防护、无备选 CPB 参数。
5. 编码器输出解码序 AU + cts/dts；容器交织、HRD 验证全部留给封装层/外部工具。
6. 运行中改码率/改 HRD 配置不支持（`reconfig` 返回 `VVENC_ERR_NOT_SUPPORTED`）。

## 8. 建议（与当前行为分离的改进项）

1. **实现 CPB 占用跟踪**：按 AU 实际比特数与声明码率模拟 CPB 充盈度，在编码循环内检测溢出/抽空并反馈给 RC（替代目前的 GOP 级近似）；这也是启用 `altCpbParamsPresent` 的前提（`SEIEncoder.cpp:192-196` 的 TODO）。
2. **消除 9 bit 计数溢出崩溃**：`cpbRemovalDelayLength` 在低延迟/长帧内周期路径下固定为 9（`SEIEncoder.cpp:109-110`），建议按 `IntraPeriod` 与 GOP 结构统一推导，或把 `CHECK` 崩溃改为自动加长字段/重新发缓冲期 SEI。
3. **统一重排序延迟来源**：dts 计算用 `m_maxTLayer`（`EncGOP.cpp:2540`）而 PT SEI 用 `numReorderPics`（`SEIEncoder.cpp:431`），建议统一改用 `sps->numReorderPics[maxTLayers-1]`，避免 GOP 结构非 CTC 时 dts 与 HRD 声明不一致。
4. **`cpbRemovalDelayDelta` 表泛化**：目前仅支持 GOP 8/16 的 CTC 结构（`SEIEncoder.cpp:119-177`），其他 GOP 尺寸在开启 PT SEI 时 `THROW`；建议按 GOP 列表动态推导 delta 表，或在配置非 CTC GOP 时自动禁用该语法并给出告警。
5. **补充 HRD 级测试**：在 `test/vvenc_unit_test` 增加 (a) 开启 RC + BP/PT SEI 的编码冒烟测试并解析 SEI 字段；(b) dts 单调性与 `dts ≤ cts` 断言；(c) 长 GOP 间距下 `auCpbRemovalDelay` 不溢出的回归用例（对应 §5.1 的崩溃路径）。
6. **支持码率重配置**：让 `reconfig()` 至少接受 `m_RCTargetBitrate` 变化，并在下一个 IRAP 重发更新后的 HRD 参数与缓冲期 SEI（目前只能重启编码器）。
7. **文档化责任边界**：在 `vvencapp`/`vvencFFapp` 文档中明确"编码器不保证 HRD 符合性，需外部验证"，避免用户把声明的 HRD 参数误当作符合性承诺。
