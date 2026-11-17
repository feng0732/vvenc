# vvenc HRD 码率缓冲时序源码分析报告

> 分析对象：本仓库 vvenc 编码器（分支 `task/31`，基线提交 `58cbc54`）
> 范围：HRD（Hypothetical Reference Decoder，假想参考解码器）参数信令、BP/PT SEI 时序、与码率控制（Rate Control, RC）及画面重排序的关系，以及编码器与封装层的责任边界。
> 方法：静态源码走读，所有结论给出 `文件:行号` 引用；测试材料引用仓库内现有单元测试。文中“当前行为”与“建议”分节列出，未做任何源码修改。

---

## 1. 总体结论与架构地图

vvenc 的 HRD 实现是一个**信令生成器，而不是缓冲模拟器**。代码库中不存在任何按比特累计的 CPB（Coded Picture Buffer）漏桶模型：没有“当前缓冲满度”变量，没有上溢/下溢检测，码率控制也不读取任何 HRD 状态。HRD 输出的全部时序值（`au_cpb_removal_delay`、`pt_dpb_output_delay` 等）均由**图像计数器和 GOP 结构静态推导**，与每帧实际产出的比特数无关。

涉及的源码组件：

| 组件 | 文件 | 职责 |
|---|---|---|
| HRD 数据结构 | `source/Lib/CommonLib/HRD.h` | `GeneralHrdParams`、`OlsHrdParams`、`HRD`（含当前 BP/PT SEI 缓存） |
| HRD 参数初始化 | `source/Lib/EncoderLib/EncHRD.cpp`、`EncHRD.h` | 由目标码率与 PTL level 表计算 `hrd_parameters()` |
| SPS 装配入口 | `source/Lib/EncoderLib/EncGOP.cpp:1404`（`xInitHrdParameters`）、`:1175`（`sps.hrdParametersPresent`） | 把 HRD 参数挂到 SPS |
| BP/PT/DUI SEI 生成 | `source/Lib/EncoderLib/SEIEncoder.cpp`、`SEIEncoder.h` | 缓冲周期、图像时序、解码单元信息 SEI 的内容计算 |
| SEI 比特流写出 | `source/Lib/EncoderLib/SEIwrite.cpp` | BP/PT/DUI SEI 语法写码 |
| HRD 语法写码 | `source/Lib/EncoderLib/VLCWriter.cpp:602`（`codeGeneralHrdparameters`）、`:626`（`codeOlsHrdParameters`）、`:977-991`（SPS 内调用） | VVC 规范语法元素落地 |
| 码率控制 | `source/Lib/EncoderLib/RateCtrl.cpp`、`RateCtrl.h` | QP/预算控制，**与 HRD 无数据通路** |
| DTS/CTS 生成 | `source/Lib/EncoderLib/EncGOP.cpp:2530-2544` | 面向封装层的解码/显示时间戳 |
| 配置与校验 | `include/vvenc/vvencCfg.h`、`source/Lib/vvenc/vvencCfg.cpp`、`source/Lib/apputils/VVEncAppCfg.h` | 开关、默认值、参数一致性检查 |
| 应用层 | `source/App/vvencFFapp/EncApp.cpp`、`source/App/vvencapp/vvencapp.cpp` | 只写字节，无任何 HRD 逻辑 |

关键开关（默认值见 `source/Lib/vvenc/vvencCfg.cpp`）：

- `m_hrdParametersPresent` 默认 **true**（`vvencCfg.cpp:612`）——SPS 里默认携带 `hrd_parameters()`。
- `m_bufferingPeriodSEIEnabled`、`m_pictureTimingSEIEnabled`、`m_decodingUnitInfoSEIEnabled` 默认 **false**（`vvencCfg.cpp:485-487`）——SEI 时序消息默认不发。
- 约束关系：无码率控制（`m_RCTargetBitrate == VVENC_RC_OFF`）时打开 BP/PT SEI 会被拒绝（`vvencCfg.cpp:786-789`）；BP/PT SEI 要求 `m_hrdParametersPresent`（`vvencCfg.cpp:2256-2257`）。

---

## 2. HRD 信令如何关联码率控制

### 2.1 唯一的数值耦合点：目标码率

`EncHRD::initHRDParameters`（`source/Lib/EncoderLib/EncHRD.cpp:67-160`）在编码器初始化时执行一次（调用链：`EncGOP::init` → `xInitHrdParameters`，`EncGOP.cpp:193, 1404-1414`），其输入与 RC 的关系如下：

```cpp
uint32_t bitRate = (uint32_t)encCfg.m_RCTargetBitrate;                    // EncHRD.cpp:77
uint32_t cpbSize = (uint32_t)profileLevelTierFeatures.getCpbSizeInBits(); // EncHRD.cpp:78
```

- **码率侧**：HRD 信令中的 `BitRate` 直接取 `m_RCTargetBitrate`，经 `bitRateScale` 缩放后写入 `bit_rate_value_minus1`（`EncHRD.cpp:99-106, 136, 147, 153`）。注意 `m_RCMaxBitrate`（封顶码率）**完全不参与** HRD 信令——全文件检索可见它只进入 `RateCtrl.cpp:326` 的 GOP 预算模型。
- **缓冲侧**：CPB 大小不取目标码率、也不取任何实测码流统计，而是取 PTL（Profile/Tier/Level）表中的 level 上限：`ProfileLevelTierFeatures::getCpbSizeInBits()` = `cpbVclFactor × level.maxCpb[tier]`（`source/Lib/CommonLib/ProfileLevelTier.cpp:206-209`；表数据在同文件 `:62-78`，例如 Level 5.1 Main tier `maxCpb=40000`，单位为 CpbVclFactor 比特）。即信令 CPB 是“该 level 允许的最大 CPB”，不是为当前码流定制的工作缓冲。
- 有无 HRD NAL/VCL 参数完全由码率是否为正决定：`generalNalHrdParamsPresent = generalVclHrdParamsPresent = (bitRate > 0)`（`EncHRD.cpp:85-88`）。
- `cbrFlag` 恒为 **false**（`EncHRD.cpp:151, 157`），即无论 RC 实际工作点多紧，信令上始终声明 VBR 漏桶。
- 所有时间层（`VVENC_MAX_TLAYER`=7，`include/vvenc/vvencCfg.h:81`）和 NAL/VCL 两套参数被初始化为相同值（`EncHRD.cpp:121-159` 注释明确说明）；`hrdCpbCntMinus1 = 0`（单 CPB，`:118`）。

### 2.2 时钟与固定帧率声明

```cpp
generalHrdParams.timeScale      = encCfg.m_FrameRate;   // EncHRD.cpp:82
generalHrdParams.numUnitsInTick = encCfg.m_FrameScale;  // EncHRD.cpp:83
```

每个 OLS 子层都写 `fixedPicRateGeneralFlag = true`、`fixedPicRateWithinCvsFlag = true`、`elementDurationInTcMinus1 = 0`、`lowDelayHrdFlag = false`（`EncHRD.cpp:129-132`）。两个值得注意的固定声明：

1. **固定帧率恒真**：即使输入时间戳存在跳帧（见 §5.3），HRD 仍声明固定帧率；可变帧率只能靠 PT SEI 里的 delta 表达，`hrd_parameters()` 本身不反映。
2. **`low_delay_hrd_flag` 恒假**：即使 `PicReordering=0` 的低延迟配置也不声明 low-delay HRD（全库唯一赋值点就是 `EncHRD.cpp:132`）。该 flag 仅在 `fixedPicRateWithinCvsFlag=false` 分支才会被写出（`VLCWriter.cpp:641-644`），而当前该分支不可达。

### 2.3 RC 侧的“缓冲”只是码率预算，不是 CPB

`RateCtrl`/`EncRCSeq`（`RateCtrl.h:101-135`）维护的是 `bitsUsed`（已用比特）、`estimatedBitUsage`（预算期望）、`maxGopRate`（GOP 级封顶，由 `min(3×targetRate, maxBitrate)` 推导，`RateCtrl.cpp:96`）等预算量，GOP 内逐帧分配 target bits 并通过 QP/lambda 收敛（`RateCtrl.cpp:1150-1269`）。这套机制：

- 不读 `EncHRD`、不读 SPS 的 CPB size，不计算漏桶满度；
- 不保证任何 AU 的累计比特满足 HRD 信令给出的 CPB 约束；
- 唯一的“过激”保护是 QP 截断（`isMeanQPLimited`，`RateCtrl.cpp:1152-1161`），属于画质/码率权衡，与 CPB 上下界无关。

`SEIEncoder.cpp:192-197` 的注释直接承认了这一缺口：

> “A commercial encoder should track the buffer state for all layers and sub-layers to ensure CPB conformance… Unfortunately VTM does not have such tracking. Thus we cannot encode alternative CPB parameters here.”

因此 `altCpbParamsPresent`、`useAltCpbParamsFlag` 恒为 false。

### 2.4 PT SEI 延迟是“帧数”，不是“比特数”

`SEIEncoder` 用两个图像计数器驱动全部 PT SEI 时序（`SEIEncoder.h:96-97`，清零于 `init()`，`SEIEncoder.cpp:66-67`）：

- `m_totalCoded[i]`：每输出一幅图像在各时间层递增（`SEIEncoder.cpp:492-495`）；
- `m_lastBPSEI[i]`：最近一次 BP SEI 时的计数器快照（`SEIEncoder.cpp:443-448`）。

顶层 `au_cpb_removal_delay` 的计算（`SEIEncoder.cpp:263`）：

```cpp
ptSei->auCpbRemovalDelay[maxNumSubLayers-1] =
    clamp(1, m_totalCoded[top] - m_lastBPSEI[top], 1 << cpbRemovalDelayLength);
```

即“距上一个缓冲周期经过了多少个画面时钟”，与该画面实际大小无关。固定帧率下这等价于等间隔移除调度，HRD 调度器据此假定每个 tick 移除一个 AU 的比特量——码率波动能否被 CPB 吸收完全取决于 §2.1 信令的 level CPB 是否够大，编码器自身不验证。

---

## 3. 画面重排序如何进入 HRD 时序

### 3.1 重排序深度的来源

`GOPCfg::xSetDBPConstraints`（`source/Lib/EncoderLib/GOPCfg.cpp:969-999`）逐 GOP 条目调用 `xGetMaxNumReorder`（`GOPCfg.cpp:1112-1129`）：统计编码顺序在前、POC 更大且时间层不高于当前条目的画面数，得到每时间层 `m_numReorderPics`，并保证低层不超过高层、`maxDecPicBuffering ≥ numReorderPics+1`。

该值经两处进入信令：

1. **SPS DPB 参数**：`sps.numReorderPics[i] = m_gopCfg->getNumReorderPics()[i]`（`EncGOP.cpp:1146-1150`），由 `HLSWriter::dpb_parameters` 写成 `dpb_max_num_reorder_pics[i]`（`VLCWriter.cpp:666-674`）。
2. **PT SEI 的输出延迟**：`picDpbOutputDelay = numReorderPics[top] + poc - m_totalCoded[top]`（`SEIEncoder.cpp:431`）。`poc` 是显示序、`m_totalCoded` 是编码序计数，二者之差即当前画面在 DPB 中的重排序等待；再叠加固定的重排序深度。
3. **BP SEI 的子层偏移**：`dpbOutputTidOffset[i]` 由 `numReorderPics[i] × 2^(maxTid-i)` 并相对顶层取模得到（`SEIEncoder.cpp:178-191`），写成 `dpb_output_tid_offset[i]`（`SEIwrite.cpp:349-355`）。

低延迟模式（`m_picReordering=0`）下 GOP 条目全部 temporal id 0（`GOPCfg.cpp:137-141`），重排序深度为 0，此时 `picDpbOutputDelay = poc - m_totalCoded`（无 B 帧重排时编码序与显示序一致，稳态为 0）。

### 3.2 延迟字段的位宽与 GOP 结构绑定

BP SEI 初始化（`SEIEncoder.cpp:70-199`）中，延迟语法元素位宽不是按内容计算，而是按配置分支硬编码：

```cpp
if (m_picReordering && m_IntraPeriod < 256) {
  bpSei.cpbRemovalDelayLength =
  bpSei.dpbOutputDelayLength  = ceilLog2(m_IntraPeriod) + 1;   // SEIEncoder.cpp:104-105（链式赋值）
} else {
  bpSei.cpbRemovalDelayLength = 9;   // 最大可表示 2^9
  bpSei.dpbOutputDelayLength  = 9;
}
```

初始 CPB 移除延迟固定为 `90000/2 = 45000`（`SEIEncoder.cpp:74`，注释“0.5 sec”，按 90 kHz 时钟假设），`initialCpbRemovalDelayLength = 16`（`:96`），NAL/VCL、所有子层同值（`:83-86`）。这是一个**与实际帧率/码率解耦的常量**：在非 90 kHz 时间基下它表示的真实时间并非 0.5 秒（例如 `FrameRate/FrameScale=25/1` 时 HRD 时钟即 25 Hz，45000 个时钟单位对应 1800 秒）。规范上延迟单位是 `timeScale` 时钟，因此该常量的注释意图只在 90 kHz 场景成立。

CPB removal delta 表只支持 CTC 的 GOP8/GOP16 两种结构（`SEIEncoder.cpp:119` `bpDeltasGOPStructure = (GOPSize==8 || GOPSize==16)`，delta 数值表在 `:123-176`，PT 侧索引表在 `:283-422`）。默认 GOP 为 32（`vvencCfg.cpp:384`），此时 `cpbRemovalDelayDeltasPresent=false`，走逐帧显式 `pt_cpb_removal_delay_minus1` 路径；若在支持表外的结构命中 delta 分支会直接 `THROW`（`:173, 309, 353` 等）。

### 3.3 面向封装层的 DTS/CTS

HRD SEI 之外，编码器还在输出 AU 上提供封装用时间戳（公共结构 `vvencAccessUnit`，`include/vvenc/vvenc.h.in:185-194`）：

- CTS 直接透传输入 `vvencYUVBuffer.cts`（`EncGOP.cpp:2533`）；
- DTS 由编码序计数推导（`EncGOP.cpp:2532, 2539-2542`）：

```cpp
iDiffFrames = m_numPicsCoded - pic.poc - pic.picsInMissing;
au.dts = ((iDiffFrames - m_maxTLayer + m_numPicsOutOffset) * ticksPerFrameMul4) / 4 + au.cts;
```

`-m_maxTLayer` 项体现重排序深度：随机访问配置下首批画面的 DTS 早于 CTS，差值恰好对应 GOP 的重排延迟。`m_ticksPerFrameMul4` 由 `TicksPerSecond × FrameScale / FrameRate` 得到（`EncGOP.cpp:228-231`）。这套时间戳与 HRD SEI 是**两条独立推导链**（一个用 `m_numPicsCoded`/POC，一个用 `m_totalCoded`/`m_lastBPSEI`），只共享“画面计数+GOP 几何”这一前提。

---

## 4. 编码器与封装层的责任划分

### 4.1 编码器（libvvenc）负责的部分

1. 在 SPS 中写 `hrd_parameters()`（general + OLS，`VLCWriter.cpp:977-991`）。单层码流 VPS 不写 HRD：`xInitVPS` 置 `eachLayerIsAnOls=true`（`EncGOP.cpp:925`），使 `vps_general_hrd_params_present_flag` 的写出分支被跳过（`VLCWriter.cpp:1236-1238`）。
2. 在 IRAP/GDR AU 的前缀 SEI 中写 BP SEI（触发条件 `EncGOP.cpp:2782-2793`：开关打开 + `isIRAP()`（定义 `Slice.h:1246`，IDR/CRA）或 GDR + 基础层 + 最终 pass + 非预分析）。
3. 每幅图像写 PT SEI（`EncGOP.cpp:2802-2808`，要求 BP 与 PT 开关同时打开）。
4. 在输出 AU 上填 `cts/dts/ctsValid/dtsValid`（`EncGOP.cpp:2530-2544`，经 `vvencimpl.cpp:755-758` 透传到 SDK）。
5. 参数集重写：`m_rewriteParamSets` 默认 true（`vvencCfg.cpp:430`），每个 IRAP 重发 VPS/DCI/SPS/PPS（`EncGOP.cpp:2565-2575`），BP SEI 也随每个 IRAP 重发并重置 `m_lastBPSEI`。

### 2.2 明确不属于编码器的部分

- **码流的发送整形（pacing）/漏桶仿真**：库内无任何代码按 HRD 调度 AU 发送时刻；编码完成即把 AU 交给调用方。
- **缓冲上/下溢防护**：无满度跟踪、无告警、无丢帧/阻塞反应。
- **容器时间戳映射**：DTS/CTS 以 `TicksPerSecond`（默认 27 MHz，`vvencCfg.h:92`）为单位给出，如何映射到 MP4 `tfdt`、MPEG-TS PCR/PTS、MKK 等由封装方负责。
- 应用层证实了这一边界：`EncApp::outputAU` 只做 `m_bitstream.write(payload, payloadUsedSize)`（`source/App/vvencFFapp/EncApp.cpp:424-433`），全 App 目录检索不到任何 HRD/CPB 处理。

### 4.3 解码单元（DU）HRD 处于结构性关闭状态

DU 级 HRD 在当前代码中不可用，证据链：

- `EncHRD.cpp:76` `useSubCpbParams = false`（注释掉的旧逻辑）→ `generalDecodingUnitHrdParamsPresent=false`（`:92`），SPS 不写 DU 参数；
- BP SEI 中 `bpDecodingUnitHrdParamsPresent=false`（`SEIEncoder.cpp:94`）；
- `EncGOP.cpp:2806` 调 PT 初始化时 `numDU` 硬编码为 1；
- 因此 `SEIEncoder.cpp:255-261` 的 DU 分支、`SEIwrite.cpp:428-441` 的 DU 语法、以及 `m_decodingUnitInfoSEIEnabled` 的实际生效路径在默认装配下均不可达（开关仍保留在 `VVEncAppCfg.h:1043`）。

---

## 5. 三类重点情形下可观察的输出时序与缓冲结果

以下“可观察结果”均可通过码流语法解析或 SDK AU 字段验证；括号内为对应源码出处。

### 5.1 缓冲接近上界/下界

**前提事实**：编码器不维护缓冲满度，因此不存在“接近上界/下界”的运行时事件，也没有日志或回调。可观察的只有两类间接现象：

1. **信令层（静态）**：CPB 上界 = PTL level 表值（`ProfileLevelTier.cpp:206-209`），初始充盈度由 BP SEI 的 `initial_cpb_removal_delay`/`_offset`（恒为 45000，`SEIEncoder.cpp:74-86`）隐含给定；移除调度由 PT SEI 逐 AU 的 `pt_cpb_removal_delay_minus1`（`SEIwrite.cpp:366`）给定。下界方向没有任何专门信令——空缓冲只能表现为解码侧等待下一个移除时刻。
2. **码流层（间接）**：RC 的预算耗尽时，`RateCtrl.cpp:1248-1253` 会把后续画面 target bits 压到 `maxGopRate` 与剩余预算 `estimatedBitUsage - bitsUsed` 的最小值内，场景切换处经 `scRelax`（`:1231-1238`）放松；极端情况下触发 mean-QP 截断（`isMeanQPLimited`，`:1152-1161`）。可观察输出是 AU 字节数变小、QP 升高，但**没有任何机制保证这足以防止 HRD 上溢**——因为 RC 不知道 CPB size 与初始延迟。单帧超大（如突发 I 帧）时，PT SEI 的 removal delay 不会因该帧变大而调整（计数器只 +1，`SEIEncoder.cpp:494`），是否合规只能由外部 HRD 校验器判定。
3. **位宽饱和是硬故障而非钳位**：若距上次 BP 的画面数超过 `1<<cpbRemovalDelayLength`，`SEIEncoder.cpp:264` 的 `CHECK` 经 `THROW` 抛异常终止编码（`CHECK` 为无条件抛出宏，`TypeDef.h:636`，非仅 Debug）。正常配置下 BP 每个 IRAP 重置计数器，IR 周期超过位宽表示范围才会触发。

### 5.2 参数集变化

当前编码器单实例只分配一个 SPS（`EncGOP.cpp:184`，注释“implementations that use more than 1 SPS need to be aware of activation issues”），HRD 参数在 `init` 时一次性计算后不再变化，因此**运行中不存在 HRD 参数集变更路径**。可观察行为围绕“重发”而非“变更”：

1. 每个 IRAP（且 `m_rewriteParamSets=true`）重发相同内容的 SPS/PPS（`EncGOP.cpp:2565-2575`），SPS 内 `hrd_parameters()` 内容逐字节相同；
2. 同一 AU 重发 BP SEI，`SEIEncoder` 将其缓存到 `m_EncHRD.bufferingPeriodSEI` 并置 `bufferingPeriodInitialized=true`（`EncGOP.cpp:2785-2792`），随后 PT 处理把 `m_lastBPSEI[i]` 重置为当前 `m_totalCoded[i]`（`SEIEncoder.cpp:443-448`）——可观察结果是该 AU 之后 PT SEI 的 `pt_cpb_removal_delay` 从 1 重新开始计数；
3. `concatenationFlag` 恒 false、`maxInitialRemovalDelayForConcatenation` 取同一常量（`SEIEncoder.cpp:92-93, 115`），拼接场景需要的替代 CPB 参数不支持（`altCpbParamsPresent=false`，`:197-198`）；
4. 配置级非法组合在 `vvenc_init_config_parameter` 阶段被拒：无 RC 开 BP/PT（`vvencCfg.cpp:786-789`）、关 HRD 却开 BP/PT（`:2256-2257`），可观察为 `vvenc_encoder_open` 返回错误而非产出码流。

### 5.3 重排序延迟与丢帧

**正常重排序（随机访问配置）**：

- 每个 AU 的 DTS ≤ CTS，二者差值在 GOP 稳态内按 GOP 几何波动，首批画面差值最大（≈ `maxTLayer` 个画面时长，`EncGOP.cpp:2540`）；
- PT SEI 的 `pt_dpb_output_delay` 随编码序/显示序差变化（`SEIEncoder.cpp:431`），DU 延迟为其 `tickDivisorMinus2+2` 倍（`:437-438`，但 DU HRD 关闭时该字段不写出，见 §4.3）；
- BP SEI 的 `dpb_output_tid_offset[]` 给出各子层相对顶层的输出偏移（`SEIEncoder.cpp:178-191`）。

**输入丢帧（CTS 跳变）**：`initPicture` 检测相邻 CTS 差 ≥ 2 个帧间隔或回退时，计算 `picOutOffset` 并累计 `m_numPicsInMissing`（`EncGOP.cpp:253-272`）。可观察结果：

1. DTS 公式经 `m_numPicsOutOffset`（`EncGOP.cpp:2535-2537`）与 `picsInMissing`（`:2532`）补偿，DTS 保持单调递增且跳空对应缺失时长，不会回退或重叠；
2. 单元测试 `checkDtsDefault` 用 `emulateMissingFrames=true` 一次性跳过 10 帧验证了该行为（`test/vvenclibtest/vvenclibtest.cpp:1044, 1063-1066, 1239-1262`），其中 `runEncoder` 断言 `AU->dts > lastDts` 恒成立、无丢帧时增量严格等于 `ctsDiff`（`:1082-1092`）；
3. 但 HRD 侧仍声明 `fixedPicRateGeneralFlag=true`（`EncHRD.cpp:129`），PT SEI 计数器对缺失画面无特殊处理——码流时序信令与真实输入节奏在此处存在表述缺口，外部 HRD 校验器若按固定帧率解释会与容器时间戳不一致。

**流末尾**：当 `(poc + GOPSize) > framesToBeEncoded` 时，PT SEI 关闭 cpb delta 编码、回退到显式延迟值（`SEIEncoder.cpp:271`），避免在不完整 GOP 上引用越界的 delta 表项；flush 阶段剩余画面经 `isFlush` 路径排空（`EncGOP.cpp:505, 811`），每幅仍产出完整 AU（由 `checkCuTreeAccessUnits` 覆盖，`vvenclibtest.cpp:1297-1322`）。

---

## 6. 现有测试材料盘点

| 材料 | 位置 | 覆盖内容 | 不覆盖什么 |
|---|---|---|---|
| 参数矩阵测试 | `test/vvenclibtest/vvenclibtest.cpp:369-372` | `m_hrdParametersPresent` 0/1 两态、BP/PT SEI 开关在 `RCTargetBitrate=0` 下被正确拒绝的校验路径 | 不解析产出码流的 HRD 语法，不验证延迟数值正确性 |
| 时间戳测试 | `vvenclibtest.cpp:1138-1206`（`checkTimestampsDefault`）、`:1239-1262`（`checkDtsDefault`） | 多种帧率（25/30/50/60/120 fps、NTSC 分数帧率）与两种 tick 模式（90 kHz、每帧 1 tick）下 DTS 单调、等间隔；丢帧补偿 | 与 HRD BP/PT SEI 无关；测试使用 `VVENC_RC_OFF`（`:1156`），此时 BP/PT SEI 本就禁止 |
| 非法组合 | `vvenclibtest.cpp:1208-1237`（`checkTimestampsInvalid`，期望失败）、`testInvalidInputParams` | 配置校验拒绝路径 | — |
| Flush/AU 完整性 | `vvenclibtest.cpp:1300-1322`（`checkCuTreeAccessUnits`） | GOP 被 IntraPeriod 截断、多线程下 AU 数量与输入帧数一致 | 不含 HRD 断言 |
| 测试码流配置 | `test/data/RTn23.cfg`（5 帧、Level 1、15 fps） | 冒烟编码 | 未启用任何 HRD SEI 选项 |
| CI | `.github/workflows/Build.yml:67, 181` | 执行 `ctest` | 仓库内无外部 HRD 一致性校验器（如 VTM 参考解码器/HRD 检查）的集成 |

结论：**现有测试验证的是时间戳工程行为和参数校验，HRD 信令本身（`hrd_parameters()` 数值、BP/PT SEI 延迟链）没有任何自动化正确性或一致性测试**。

---

## 7. 当前行为汇总（事实陈述）

1. HRD 是纯信令模块：CPB 尺寸取 PTL level 上限（`EncHRD.cpp:78`），码率取 `m_RCTargetBitrate`（`:77`），`m_RCMaxBitrate` 不进信令；无漏桶仿真、无满度跟踪、无上下溢反应。
2. RC 与 HRD 无数据通路，仅共享配置项 `m_RCTargetBitrate`；RC 的预算/QP 机制不保证 HRD 合规（`SEIEncoder.cpp:192-197` 注释自认）。
3. `cbrFlag` 恒 false、`lowDelayHrdFlag` 恒 false、`fixedPicRate*` 恒 true、替代 CPB 参数与 DU HRD 结构性关闭（`EncHRD.cpp:92, 129-132, 151`；`SEIEncoder.cpp:94`；`EncGOP.cpp:2806`）。
4. 全部 PT 时序由画面计数器推导（`SEIEncoder.cpp:263, 431, 494`），与帧大小无关；延迟位宽按 GOP/IR 配置硬编码（`:101-111`），溢出即抛异常（`:264`）。
5. 初始 CPB 延迟为与帧率脱钩的常量 45000（`:74`）；CPB delta 表仅支持 GOP8/16（`:119-176`），默认 GOP32 走显式延迟路径。
6. 重排序深度由 GOP 几何静态计算（`GOPCfg.cpp:969-999, 1112-1129`），同时进入 SPS DPB 参数、PT 输出延迟、BP tid 偏移和 SDK DTS 四条信令链。
7. 参数集单实例不变，仅在 IRAP 重发；BP SEI 随 IRAP 重置 PT 计数基准（`EncGOP.cpp:2565-2575, 2782-2793`；`SEIEncoder.cpp:443-448`）。
8. 封装责任（pacing、容器时间戳映射、合规校验）全部在库外；App 层只写字节（`EncApp.cpp:424-433`）。
9. 丢帧时 DTS 有补偿且单调（有测试覆盖），但 HRD 固定帧率声明不变，两套时序表述存在缺口。

---

## 8. 改进建议（与上述事实分离，供评估，未改动代码）

按优先级排列，括号内为对应现状条目：

1. **增加真正的 CPB 漏桶模型（最高优先级）**。在 RC 反馈环中按 SPS 信令的 `BitRate`/`CpbSize` 和 BP 初始延迟维护每子层缓冲满度，AU 入桶加实际比特、按 tick 泄漏；将“预计上溢”作为 QP 收紧/帧级目标码率下调的额外约束输入。这是闭合“RC 不保证 HRD 合规”缺口（§7.1-7.2）的根本手段，也是启用 `altCpbParamsPresent` 的前提（`SEIEncoder.cpp:192-197`）。
2. **CPB 尺寸与码率联动可选**。当前一律信令 level 最大 CPB（§7.1），对低码率码流意味着声明了远超需要的缓冲，下游无法据此推断真实启动延迟。建议增加配置项允许按目标码率 × 期望延迟（如 1 s）定制 CPB，并做 level 上限钳制。
3. **初始延迟常量化修正**。`45000` 应按实际 `timeScale` 换算（0.5 s × `timeScale`），或暴露为配置项；同时修正“0.5 sec”注释的适用范围说明（§7.5）。
4. **VBR/CBR 信令与 RC 模式对齐**。当用户意图 CBR（或未来实现缓冲约束后）时置 `cbrFlag=true`；低延迟配置（`m_picReordering=0`）下评估置 `lowDelayHrdFlag=true` 并打通 `fixedPicRateWithinCvsFlag=false` 的写出分支（§7.3）。
5. **补齐 HRD 自动化测试**。在 `vvenclibtest` 中增加：启用 BP/PT SEI 的 RC 编码用例，解析产出 SPS/SEI 并断言（a）`pt_cpb_removal_delay` 序列与画面计数一致、（b）BP 后计数器复位、（c）`pt_dpb_output_delay` 与 `numReorderPics` 一致；条件允许时接入外部参考 HRD 校验器做端到端合规（§6）。
6. **delta 表与默认 GOP 对齐或泛化**。默认 GOP32 不支持 CPB delta 压缩（§7.5），建议要么补 GOP32 表，要么在文档/help 中显式说明该限制，并避免用户在 GOP8/16 之外的配置上误开相关优化。
7. **丢帧场景的 HRD 表述**。固定帧率声明与 CTS 跳变共存时（§7.9），考虑在 PT SEI 中利用现有 delta 机制表达非均匀间隔，或在文档中要求封装层以容器时间戳为准、HRD 时序仅适用于标称帧率。
8. **DU HRD 去留决策**。当前保留开关与大量死代码（§4.3），建议要么明确标注不支持并从 help 隐藏，要么补齐 `useSubCpbParams`、`numDU>1` 的完整通路，减少维护歧义。

---

## 附：关键源码索引

| 主题 | 入口 |
|---|---|
| HRD 参数计算 | `source/Lib/EncoderLib/EncHRD.cpp:67-160` |
| SPS 挂载 HRD | `source/Lib/EncoderLib/EncGOP.cpp:1404-1414, 1175` |
| HRD 语法写码 | `source/Lib/EncoderLib/VLCWriter.cpp:602-664, 977-991` |
| BP SEI 生成 | `source/Lib/EncoderLib/SEIEncoder.cpp:70-199` |
| PT SEI 生成 | `source/Lib/EncoderLib/SEIEncoder.cpp:236-496` |
| SEI 触发条件 | `source/Lib/EncoderLib/EncGOP.cpp:2775-2808` |
| SEI 语法写码 | `source/Lib/EncoderLib/SEIwrite.cpp:250-441` |
| 重排序深度 | `source/Lib/EncoderLib/GOPCfg.cpp:969-999, 1112-1129` |
| DTS/CTS | `source/Lib/EncoderLib/EncGOP.cpp:248-272, 2530-2544` |
| RC 预算模型 | `source/Lib/EncoderLib/RateCtrl.cpp:90-96, 1150-1269` |
| PTL CPB 表 | `source/Lib/CommonLib/ProfileLevelTier.cpp:62-78, 206-209` |
| 配置校验 | `source/Lib/vvenc/vvencCfg.cpp:786-789, 2256-2257` |
| 时间戳测试 | `test/vvenclibtest/vvenclibtest.cpp:1017-1136, 1239-1262` |
