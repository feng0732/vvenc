# VVenC 两遍码率控制在场景切换处的行为分析

分析对象：`source/Lib/EncoderLib/RateCtrl.{h,cpp}`，辅以 `EncLib.cpp`、`EncGOP.cpp`、`GOPCfg.cpp`、`PreProcess.cpp`、`EncHRD.cpp`、`vvencCfg.cpp`。

**阅读约定**：本文严格区分

- **当前行为** —— 直接由源码或源码级实测得出，可复现；
- **建议** —— 作者观点，未在仓库中实现。

**证据来源**：源码阅读 + 在 80×44、15 fps、≤300 帧的小素材上跑两遍 RC（每例秒级，未做长时编码）。内部状态（`flushPOC`、`isEndOfSequence`、`dLimit`、`tmpTargetBits`、场景切换检测结果）通过**临时**插桩观察，插桩已用 `git checkout` 撤销，工作区为干净状态（`git status --porcelain` 为空）。报告中的绝对码率数字均来自**干净构建**的二进制。

---

## 0. 两遍 RC 的骨架（先建立共识）

| 环节 | 位置 | 关键事实 |
|---|---|---|
| 第一遍配置 | `EncLib.cpp:385` `m_firstPassCfg.m_RCTargetBitrate = 0` | 第一遍是**固定 QP 编码**，不是"低码率预编码"；码率由 `rcQP`（`vvencCfg.cpp:838`）或 `RCInitialQP` 决定 |
| 第一遍统计落盘 | `RateCtrl.cpp:467-561` `storeStatsData()` | 每帧一行 JSON：`numBits/qp/lambda/visActY/psnrY/isIntra/tempLayer/isStartOfIntra/isStartOfGop/gopNum/scType/spVisAct` |
| 第二遍读入 | `RateCtrl.cpp:437-464, 564-624` | 读 header + 逐帧；`IntraPeriod/GOPSize/CTUSize` 不符只**告警不失败**（`:458-463`） |
| 统计 → 目标 | `RateCtrl.cpp:925-1035` `processGops()` | 全局比例 + GOP 内比例 + 预封顶 |
| 逐帧目标/QP | `RateCtrl.cpp:1165-1325` `initRateControlPic()` | 虚拟预算修正 + QP 模型 + QP 平滑 |
| 编码后回填 | `RateCtrl.cpp:1150-1163` `updateAfterPicEncRC()` | 更新 `bitsUsed` / `estimatedBitUsage` / 逐 TL 的 QP 校正 |

第一遍与第二遍唯一的"契约"是：**第一遍给出每帧的相对比特量与 R-D 锚点（`qp`、`lambda`），第二遍用一个全局标量把它们缩放到目标码率**，再叠加逐帧的预算反馈。下面把这个链条拆开。

---

## 1. 第一遍统计与第二遍目标码率的对应关系

### 1.1 当前行为：四步映射

**第 1 步 · 第一遍平均每帧比特 `bp1pf`**（`RateCtrl.cpp:810-875`）

两遍模式下走 `:856-874` 的分支：按帧的 `visActY`（时间活动度）与 `spVisAct`（空间活动度）之差做**活动度加权平均**，并对极暗帧做衰减（`:867`）。返回值：

```cpp
return totalBitsFirstPass / (double) m_listRCFirstPassStats.size();   // :874
```

注意 `:819` 的时域层加权分支（`getAverageBitsFromFirstPass` 上半段，含 `gopsInIp`、`tlBits[8]`）**只在 `m_LookAhead` 为真时生效**；两遍模式不进入。也就是说，两遍 RC 的第一遍平均**不做时域层归一化**，完全依赖第一遍的实际帧序列。

**第 2 步 · 全局比例 `ratio`**（`RateCtrl.cpp:927-928`）

```cpp
const double bp1pf = getAverageBitsFromFirstPass();
const double ratio = (double) encRCSeq->targetRate / (encRCSeq->frameRate * bp1pf); // 2nd-to-1st pass
```

`ratio` 的物理含义就是 **目标码率 / 第一遍实测码率**。它作用于每一帧：

```cpp
it->targetBits = (int) std::max (1.0, 0.5 + it->numBits * ratio);   // :946
```

因此第一遍的**帧间相对比特分布被完整保留**，第二遍只改变整体电平。

**第 3 步 · GOP 内份额 `frameInGopRatio`**（`RateCtrl.cpp:933-976`）

按 `gopNum` 把帧分桶求和得 `gopBits[]`（`:933` 的 `+2` 是给首个 I 帧单独成桶和末尾不完整 GOP 预留），然后

```cpp
it->frameInGopRatio = it->targetBits * fac;   // = targetBits / gopBits[gop]   :968
```

注释写明这是 VCIP'21 论文里的 `rf/gf`。**首个 I 帧被单独放进一个桶**（`:949-952`、`:970-975`），其 `gopTempVal` 被设为 `1.0`，注释为 "relax rate constraint a bit in first 2 GOPs"。

**第 4 步 · 预封顶（pre-capper）**（`RateCtrl.cpp:978-1034`）

仅当 `maxGopRate < INT32_MAX` 时执行。`maxGopRate` 在 `:96` 定义为

```cpp
maxGopRate = min(3*targetRate, maxBitrate) * GOPSize / frameRate
```

即 **每 GOP 的比特上限**，`MaxBitrate` 缺省被 `vvencCfg.cpp:794-800` 置为 `INT32_MAX`，用户给负值时按 `-x/16` 解释为 `targetRate` 的倍数（`vvencCfg.cpp:798-800`）。预封顶逐桶比较

```cpp
maxBits = scale * (1.0 + gopTempVal);                          // :987-989
if (gopBits > maxBits) gopTempVal = maxBits / gopBits;         // :991-996
```

被削掉的比特再"平均"分给未封顶的 GOP：

```cpp
savedBits /= gopBits.size() - savedGOPs;                       // :1005
```

**第 5 步（第二遍运行时）· 逐帧预算修正**（`RateCtrl.cpp:1245-1272`）

```cpp
d = tmpTargetBits
  + min( (int64_t)maxGopRate, estimatedBitUsage - bitsUsed ) * tmpVal * frameInGopRatio;   // :1248
```

`estimatedBitUsage - bitsUsed` 是**虚拟预算结余**：`estimatedBitUsage` 累加"每帧算出的目标"，`bitsUsed` 累加"实际花掉"（`:120-134`）。两者之差被 `maxGopRate` 双向截断，再乘 `tmpVal`（`:1235`，IGOP 时 `0.5+0.5*lastIntraSM`，否则 `0.5`）与 `frameInGopRatio`。之后：

- **末段（EOS）**：`:1251-1254` 追加 `(1.0 - tmpVal)` 份额，注释为 "try to hit target rate more aggressively in last coded frames"；
- **非 EOS**：用 `dLimit` 双向夹住（`:1255-1262`），`dLimit` 见 `:1186-1189`；
- **GOP 起点的码率封顶**：`:1264-1270`。

最后 `sliceQP` 由第一遍的 `qp` 与 `lambda` 推出（`:1274-1277`、`:1288`）：

```cpp
d = firstPassSliceQP - (105.0/128.0)*sqrt(firstPassSliceQP)*log2( d / it->numBits );   // :1276
sliceQP = int( 0.5 + d + (isIntra ? 0.375 : 0.5)*max(0.0, tmpVal - d) + qpCorrection ); // :1277
lambda  = it->lambda * pow( 2.0, (sliceQP - firstPassSliceQP)/3.0 );                    // :1288
```

**结论**：第一遍与第二遍的对应关系是

> `第二遍每帧目标 = 第一遍该帧比特 × (目标码率 / 第一遍平均码率)`，再按 GOP 份额和虚拟预算结余逐帧修正；第二遍的 QP 由第一遍 QP 经 `d/numBits` 的对数模型平移得到。

这是一条**纯比例 + 对数 R-D 模型**的映射，没有内容类别、没有失真约束、也没有 GOP 级缓冲占用量。

### 1.2 该关系对 GOP 决策的影响（当前行为）

1. **GOP 只是"分桶的记账单位"**。`gopNum` 是 `GOPCfg::getNextGopEntry()` 在每完成一个 GOP 时自增的计数（`GOPCfg.cpp:178, 230`）。`processGops()` 从不改变 GOP 结构、不改变 I 帧位置、不改变时域层分配 —— 它只决定"每个 GOP 拿多少比特"。
2. **GOP 内比特分配权完全留给第一遍**。`frameInGopRatio` 直接等于第一遍的帧比特占比，所以如果第一遍在高 QP 下把某个 GOP 的 B 帧压到接近 0，第二遍也会按这个比例分配，[`clipTargetQP()`](source/Lib/EncoderLib/RateCtrl.cpp) 只能事后微调。
3. **首个 I 帧独占一桶，切断了 I 帧与同 GOP 内 P/B 帧的互相补偿**（`:949-952`、`:970-975`）。好处是 I 帧不会挤占同 GOP 的预算；代价是短片段里 I 帧的绝对开销**无法被后续帧摊薄**。
4. **场景切换被转成 "refresh" 标志，而不是 GOP 决策**。`detectSceneCuts()`（`:877-923`）算 `isNewScene`，三条件任一成立：

   ```cpp
   it->isNewScene = ((it->visActY*64 > visActPrev*181)
                  || (isTL01 && it->visActY <= (1u << (bitDepth-6)))
                  || (isTL01 && it->visActY+3 > visActPrev && psnrTL01Prev > 0.0
                      && abs(it->psnrY - psnrTL01Prev) > 4.5));                 // :895
   ```

   并用 `minPocDif = (gopSize+1)>>1`（`:879`）做过近抑制（`:897-911`）；STA 产生的 I 帧（`scType == SCT_TL0_SCENE_CUT`）被当作场景切换的代理（`:912-915`）。检测结果只写进 `it->refreshParameters`（`:917`），在第二遍用来：
   - 复位 `qpCorrection[]`（`:1196-1199`）；
   - 放宽 QP 变化幅度（`:220`，`clipRange` 从 `max(3, 6-(frameLevel>>1))` 变为 `5 + (intraPeriod + gopSize/2)/gopSize`）；
   - 触发 `scRelax` 以放宽后续帧的约束（`:1227-1231`）。

   **它并不会让 RC 提前结束当前 GOP、插入 I 帧或重新分配 GOP 预算。** 真正的 I 帧插入由 `PreProcess::xDetectSTA()`（`PreProcess.cpp:466-468`）完成，RC 只是"被告知"。

### 1.3 该关系对"缓冲约束"的影响（当前行为）

**仓库里没有 CPB/HRD 缓冲模型。** `EncHRD::initHRDParameters()`（`EncHRD.cpp:69-161`）只把 `bitRate` 和 `cpbSize` 写进 SPS/SEI 参数，`cbrFlag` 固定为 `false`（`:151, 157`），没有满度跟踪，也不参与任何编码决策；`EncGOP` 只在 `:1404-1412`、`:2783-2789` 使用它。

RC 里充当"缓冲约束"的是三个量：

| 约束 | 位置 | 语义 |
|---|---|---|
| `maxGopRate` | `:96` | GOP 级比特上限，同时用作预算结余的**双向截断**（`:1248`） |
| `dLimit` | `:1189` | 单帧相对 `tmpTargetBits` 的上下浮动倍数，`isRateCapperMax` 时 3.0，否则 `0.5*budgetRelaxScale+0.5` ∈ {1.5, 2.0} |
| `lastIntraSM` | `:1205`、`:1266` | 时间平稳性度量（I 帧占其 GOP 的比特比），进入 GOP 起点码率封顶 |

其中 `dLimit` 的取值由 `budgetRelaxScale`（`:1186`）和 `isRateCapperMax`（`:1187`）决定：

```cpp
budgetRelaxScale = ( maxGopRate + 0.5 < 2.0 * targetRate * gopSize / frameRate ) ? 2 : 3;
isRateCapperMax  = ( maxGopRate + 0.5 >= 3.0 * targetRate * gopSize / frameRate );
dLimit           = isRateCapperMax ? 3.0 : 0.5 * budgetRelaxScale + 0.5;
```

即 **`MaxBitrate` 越松 → `dLimit` 越大 → 单帧允许偏离目标越多**；`MaxBitrate` 越紧 → `dLimit` 越小 → 单帧被夹得越死（但见 §3.2 的反常结果）。

**建议**：
- 若下游确有 CPB 约束，应在 `initRateControlPic()` 的 `d` 计算处引入真正的缓冲满度项（对 `bitsUsed` 做滑动窗峰值约束），而不是只靠 `maxGopRate` 的 GOP 内均值约束；
- `dLimit` 与 `budgetRelaxScale` 的耦合让"最大码率"同时控制了"码率平滑度"，语义重叠，建议拆成两个参数。

---

## 2. GOP 决策与缓冲约束如何造成"短片段"与"码率额度不足"

### 2.1 短片段：EOS 窗口是**固定 32 帧**，不是序列比例

**当前行为。** `xProcessFirstPassData()` 无条件把最后 `max(32, GOPSize)` 帧标为末段：

```cpp
flushPOC = m_listRCFirstPassStats.back().poc - std::max( 32, m_pcEncCfg->m_GOPSize );   // :786
```

注意字面量 `32` 而不是 `VVENC_MAX_GOP`（=64，`include/vvenc/vvencCfg.h:79`）。第二遍据此判定

```cpp
const bool isEndOfSequence = ( it->poc >= flushPOC && flushPOC >= 0 );   // :1188
```

EOS 标记会改变**四条互不相干的逻辑**：

| 影响 | 位置 | 短片段后果 |
|---|---|---|
| 跳过 `dLimit` 双向夹 | `:1255-1262`（`if (isEndOfSequence)` 走另一支） | 单帧可任意偏离目标 |
| 追加 `(1-tmpVal)` 份额 | `:1250-1254` | 预算结余被**全额**压到最后这些帧 |
| 关闭 IGOP 平滑 | `:1225` `isIntraGOP = (... && !isEndOfSequence ...)` | 窗口内 I/P/B 关系退化为 `tmpVal=0.5` |
| 关闭 GOP 起点码率封顶 | `:1264` `if ( !isEndOfSequence && it->isStartOfGOP ... )` | 峰值码率不再受限 |

所以**只要帧数 ≥ 33（GOPSize ≤ 32 时），序列尾部 32 帧全部进入 EOS 模式**；对 33 帧片段就是 100%。

实测（`noise300.yuv`，80×44，15 fps，GOP32，`-ip 32`，目标 100 kbps）：

| N | 实测码率 | I 帧 | B 帧 | 码流字节 |
|---:|---:|---:|---:|---:|
| 32 | **210.16 kbps** | 423.84 kbps (QP 25.00) | 203.27 kbps (QP 39.27) | 56 043 |
| 33 | **167.11 kbps** | 423.36 kbps (QP 25.00) | 159.10 kbps (QP 42.11) | 45 954 |

仅多 1 帧（poc 32，624 bits，QP 52），整段平均码率从 210 掉到 167 kbps（−20.5%）。用 GOP16 复测（EOS 窗口仍为 32）：N=32 → 177.90 kbps，N=33 → 123.08 kbps（−30.8%）。对 300 帧的噪声片段，尾部 33 帧（poc 267–299）平均 **547 bits/帧、QP 61.3（25/33 帧 QP=63）**，而主体平均 8 262 bits/帧、QP 46.0 —— 相差 15 倍。

**这就是"短片段"问题的根**：EOS 窗口长度与片段长度无关联。片段越短，被 EOS 特殊处理覆盖的比例越大；33–64 帧的片段的**全部**比特决策都运行在"末段激进模式"下。

**建议**：
1. EOS 窗口应随**实际编码长度**与 `IntraPeriod` 缩放，例如 `min(N, max(GOPSize, intraPeriod))`，而不是固定 32；
2. `flushPOC` 应只在 `flush == true`（即真的收到 flush、确认序列结束）时设置。当前 `if ( flush || !m_pcEncCfg->m_LookAhead )`（`:783`）让两遍模式**在常规流程里**就设置了 EOS，这是两遍路径特有的行为（单遍 look-ahead 只在 `flush` 时进入 `:788-791` 的 `isRateSavingMode`）；
3. `32` 字面量应改为具名常量，且 `GOPSize < 32` 时窗口不应大于一个 `IntraPeriod`。

### 2.2 短片段：GOP 桶结构与 I 帧独占桶

**当前行为。** `gopBits` 的桶数是 `2 + (末帧 gopNum - 首帧 gopNum)`（`:933`），首个 I 帧（`poc==0 && isIntra`）额外占一桶（`:949-952`、`:1026-1031`）。对**不足一个 GOP** 的片段，`processGops()` 只有 1 个有效桶 + 2 个空桶，于是：

- `frameInGopRatio` 的分母是一个 GOP 内**尚未完整出现**的比特和，比例本身仍然可用（因为分子分母同乘 `ratio`），但语义上的 `rf/gf` 已失真；
- 预封顶（`:978-1034`）对单桶片段几乎无效（见 §3.2 实测）；
- 逐帧预算修正里的 `lastIntraSM`（`:1205`）在只看得到 1 个 I 帧时缺乏统计意义。

CI 里正是这种配置：`cmake/modules/vvencTests.cmake:93-95` 用 `-f 5 --Bitrate=10000`，`test/data/RTn23.cfg:9` 也只有 5 帧。实测该用例（目标 10 kbps）：

```
avg_bitrate = 17.90 kbps   (超出 79%)
I 帧 81.12 kbps (5408 bits, QP 30)   B 帧 2.10 kbps (QP 50.75)
```

单是 I 帧就花掉 5 帧预算（10 000/15×5 = 3 333 bits）的 1.6 倍。I 帧独占桶意味着**没有后续帧可以补偿它**。

**建议**：
- 对 `N < GOPSize` 的片段，把 I 帧并入首个 GOP 桶（而不是独占），让 `frameInGopRatio` 至少反映片段内的真实占比；
- 在片段短于一个 GOP 时，只用 `ratio` 做全局缩放并关闭预封顶，避免单桶下的 `frameInGopRatio` 语义漂移。

### 2.3 码率额度不足：QP 饱和 + 预算深度为负时 `dLimit` 反向生效

**当前行为（额度不足的"正常"表现）。** 当目标码率低于内容可达码率时，`clipTargetQP()` 把 QP 抬到 `MAX_QP`：

```cpp
qp = Clip3( frameLevel + std::max(0, baseQP>>1), MAX_QP, qp );    // :216
```

实测（`noise300.yuv`，96 帧，GOP32，`-ip 32`）：

| 目标 | 实测 | QP 饱和情况 |
|---:|---:|---|
| 5 kbps | 3.01 kbps | QP=63 于 62/96 帧 |
| 10 kbps | 3.81 kbps | QP=63 于 24/96 帧 |
| 20 kbps | 9.62 kbps | 最高 QP 58 |

即额度严重不足时 RC **如实下穿**目标（不会超发），代价是整段 QP 饱和、质量崩塌。这里没有崩溃，只是"尽力而为"。

**当前行为（额度不足 + EOS 叠加，产生反直觉结果）。** 当预算结余 `estimatedBitUsage - bitsUsed` 深度为负时，`:1248` 的 `d` 变成大负数，随后被下界钳住：

```cpp
else if ( d * dLimit < pic.encRCPic->tmpTargetBits )
  d = pic.encRCPic->tmpTargetBits / dLimit;      // :1259-1262
```

于是**输出的单帧目标实际上由 `tmpTargetBits / dLimit` 决定**。而 `dLimit` 随 `MaxBitrate` 收紧从 3.0 掉到 1.5（见 §1.3）——**收紧最大码率反而抬高了下界**。实测（`noise300.yuv`，300 帧，目标 100 kbps，`-ip 32`）：

| `MaxBitrate` | 倍数 | 实测码率 | QP 均值 | QP=63 帧数 |
|---|---:|---:|---:|---:|
| 缺省 | 3× | **111.20 kbps** | 47.7 | 25 |
| `-24` | 1.5× | **151.79 kbps** | 44.4 | 18 |
| `-30` | 1.875× | **164.70 kbps** | 43.5 | 20 |
| `-36` | 2.25× | 140.68 kbps | 45.3 | 22 |
| `-48` | 3× | 111.20 kbps | 47.7 | 25 |

收紧到 1.5× 让码率上升 36.5%。插桩确认了机制：EOS 之外的 267 帧里上界钳一次都没触发，下界钳触发 227 次（3×）/ 241 次（1.5×），`d` 的原始和从 314 835 bits 升到 426 293 bits。该现象**依赖内容**：在自然素材 `c300` 上同样收紧反而降到 56.91 kbps（缺省 71.46），说明它只在"预算深度赤字"时出现 —— 而短片段恰好普遍处于赤字状态。

**建议**：
- 把"最大码率"与"单帧偏离上限"解耦：`dLimit` 不应由 `isRateCapperMax` / `budgetRelaxScale` 推导，而应是独立参数；至少在 `d` 已被下界钳住时不要再让 `dLimit` 参与；
- 当预算赤字时，应让 `d` 单调不增于 `maxGopRate`（当前 `maxGopRate` 同时出现在 `:1248` 的截断和 `dLimit` 的推导里，方向相反）；
- 对"额度不足"给出显式信号（日志/统计），而不是静默地让 QP 饱和 —— 当前唯一的可见迹象是逐帧日志里的 `QP 63`。

---

## 3. 两个边界场景的可观察结果

### 3.1 边界 A：N=32 → N=33 的末段窗口台阶

**设置**：`noise300.yuv`（随机噪声，80×44，15 fps，GOP32，`-ip 32`），目标 100 kbps，两遍 RC。

| N | 实测码率 | 占目标 | 逐 GOP 实测比特 |
|---:|---:|---:|---|
| 32 | 210.16 kbps | 210% | 单 GOP = 448 344 |
| 33 | 167.11 kbps | 167% | 单 GOP = 367 632（poc 0–30 全部升高 QP） |

poc 0–30 的每一帧在 N=33 时 QP 都更高（如 poc 20：9 976/44 → 5 088/49），只有 I 帧（poc 31）不变（28 256/25 → 28 224/25）。原因：N=33 时 `flushPOC = 32 - 32 = 0`，`isEndOfSequence` 对**所有**帧为真（`:1188`），`:1250-1254` 的 `(1-tmpVal)` 份额把全段赤字提前摊掉。N=32 时 `flushPOC = -1`，`flushPOC >= 0` 不成立，EOS 逻辑整体不激活。

同一现象在 GOP16 下复现（EOS 窗口仍是 32）：177.90 kbps → 123.08 kbps。

**对照（自然素材 `c300`，同一目标 100 kbps）**：

| N | 32 | 33 | 45 | 64 | 128 | 300 |
|---|---:|---:|---:|---:|---:|---:|
| 实测 kbps | 49.29 | 49.17 | 53.60 | 60.27 | 66.06 | 71.46 |

自然素材上台阶不明显，因为内容本身在 QP 下限附近（QP 7–16），RC 靠 QP 就能吸收；噪声素材无此余地，赤字只能由 EOS 逻辑集中兑现。

**源码证据**：`RateCtrl.cpp:783-787`（`flushPOC` 定义）、`:1188`（EOS 判定）、`:1225`、`:1250-1262`、`:1264`。
**测试证据**：仓库**没有**覆盖该边界的用例。`cmake/modules/vvencTests.cmake:93-99` 的 4 个 `rc2p` 用例全部是 `-f 5`，且只做 `compare_files`（码流字节一致）比较，**不校验码率**。`ctest -R rc2p` 全部通过（10/10），但对本节现象零覆盖。

### 3.2 边界 B：`MaxBitrate` 收紧反而抬高码率 + 预算被压到尾部

**设置**：`noise300.yuv`，300 帧，GOP32，`-ip 32`，目标 100 kbps。

| 场景 | 实测码率 | 尾部 32 帧 (poc≥267) | 主体 267 帧 |
|---|---:|---:|---:|
| `MaxBitrate` 缺省（3×） | 111.20 kbps | 平均 **547 bits/帧**，QP 61.3，25/33 帧 QP=63 | 8 262 bits/帧，QP 46.0 |
| `MaxBitrate` 1.5× | 151.79 kbps | 平均 **786 bits/帧**，QP 60.0 | 11 273 bits/帧，QP 42.4 |

两端都偏离目标：缺省配置下主体超发 123.9 kbps、尾部只发 8.2 kbps（比值 0.07）；收紧上限后**整体**更超发（+36.5%），尾部反而略升。

预封顶的诊断数据（插桩观察，已撤销）：`processGops()` 在 `:1003-1005` 把削减量**平均**分给未封顶 GOP，而 `:1013-1021` 只有在遇到 I 帧时才重算 `scale`，非 I 帧的 GOP 起点沿用上一个 `scale`。实测 `afterPrecap` 目标总和：1.5× 时为 1 759 997 bits（比预算 2 000 000 少 12%），3× 时恰为 1 999 998 bits。也就是说**预封顶本身的方向是对的（少给 12%），但运行时 `dLimit` 反向补偿把它吃回去还倒灌**。

**场景 B 的第二个可观察面：单一 GOP 片段上预封顶完全失效。** 对 N=32 的片段（仅一个 GOP 桶），缺省 210.16 kbps 与 1.5× 222.93 kbps 几乎无差别（且方向相反）；对 N=64 是 134.99 → 148.68 kbps。这与 `:1003-1005` 的"平均分给未封顶 GOP"在桶数不足时的退化一致。

**源码证据**：`RateCtrl.cpp:96`（`maxGopRate`）、`:978-1034`（预封顶）、`:1003-1005`（平均分配）、`:1013-1021`（`scale` 仅在 I 帧重算）、`:1186-1189`（`dLimit`）、`:1248-1262`（预算结余与 `dLimit`）、`:1264-1270`（GOP 起点封顶）。
**测试证据**：`cfg/rc2p.cfg` 只设 `TargetBitrate/NumPasses/RCInitialQP`，不含 `MaxBitrate`；`cmake/modules/vvencTests.cmake` 中没有任何 `--MaxBitrate` 用例。本节行为无回归保护。

### 3.3 补充可观察结果：第一遍 QP 的选取会影响整条链路

**当前行为。** `rcQP` 只由**分辨率**和**目标码率**决定（`vvencCfg.cpp:837-838`）：

```cpp
const int rcQP = (RCInitialQP > 0 ? min(MAX_QP, RCInitialQP)
                 : max(0, MAX_QP_INIT_QPA - (FirstPassMode > 2 ? 4 : 2)
                          - int(0.5 + sqrt((d * max(0, RCTargetBitrate)) / 500000.0))));
```

在 80×44、目标 1 Mbps 时 `d = 8294400/3520 ≈ 2356`，`sqrt(2356×1e6/5e5) ≈ 68.6`，结果被 `max(0, …)` 钳到 **0**。实测第一遍 JSON 的 `qp` 落在 **−3 … 6**（负数来自 GOP 内 QP offset），即第一遍近乎无损，其比特量对第二遍的目标没有代表性 —— 此时 `ratio` 达 5.32，`getBaseQP()`（`:330-374`）也失去校准价值。指定 `RCInitialQP=40` 后第一遍 QP 变为 36…50，行为正常。

对比三档目标（同一 45 帧自然片段）：

| 目标 | 第一遍实测 | 第一遍 QP | 第二遍实测 |
|---:|---:|---|---:|
| 40 kbps | 7.8 kbps | 23–34 | 29.87 kbps |
| 100 kbps | 26.9 kbps | 15–24 | 53.60 kbps |
| 1000 kbps | 188.0 kbps | **−3…6** | 111.30 kbps |

**建议**：`rcQP` 的下界应设为一个非零地板（例如 `max(12, …)`），或在小分辨率/高目标码率时自动上调 `FirstPassMode`（`vvencCfg.cpp:826-833` 已有降采样开关）以保持第一遍在合理 QP 区间；同时在 `getBaseQP()` 里检测"第一遍 QP 触底"并退回无第一遍数据的经验公式（`:366-371`）。

---

## 4. 当前行为 vs 建议（汇总）

### 4.1 当前行为（可直接引用/复现）

| # | 行为 | 位置 | 可观察后果 |
|---|---|---|---|
| A | 两遍映射 = 全局 `ratio` × 第一遍帧比特，帧间相对分布原样保留 | `RateCtrl.cpp:928, 946, 968` | 第一遍的 GOP 内失真分配被第二遍继承 |
| B | GOP 仅作为记账桶；场景切换只写 `refreshParameters` | `:925-1035`, `:877-923` | RC 不会因切场景重排 GOP 或预算；I 帧插入在 `PreProcess.cpp:466-468` |
| C | 无 CPB/HRD 缓冲模型，`maxGopRate` + `dLimit` + 虚拟预算结余替代 | `EncHRD.cpp:69-161`, `:1189, 1248` | 峰值约束只在 GOP 均值意义下存在 |
| D | 末段窗口固定 `max(32, GOPSize)`，且两遍模式在常规流程即激活 | `:783-787` | N≥33 的片段尾部 32 帧全进 EOS 特殊路径 |
| E | EOS 路径跳过 `dLimit` 与 GOP 起点的码率封顶，并把赤字全部兑现 | `:1225, 1250-1262, 1264` | N=32→33 码率台阶 210→167 kbps（噪声素材） |
| F | 首个 I 帧独占 GOP 桶 | `:949-952, 1026-1031` | 短片段 I 帧开销无法摊薄；CI 5 帧用例超目标 79% |
| G | 预封顶按"平均"再分配，`scale` 仅在 I 帧重算 | `:1003-1005, 1013-1021` | 单 GOP 片段上封顶几乎无效 |
| H | 预算赤字时单帧目标由 `tmpTarget/dLimit` 决定 | `:1259-1262` | 收紧 `MaxBitrate`(3×→1.5×) 使码率从 111→152 kbps |
| I | 额度不足时 QP 饱和到 63，如实下穿目标 | `:216` `Clip3(..., MAX_QP, qp)` | 目标 5 kbps → 实测 3.01 kbps，62/96 帧 QP=63 |
| J | 第一遍 `rcQP` 无下界地板 | `vvencCfg.cpp:838` | 小分辨率/高目标时第一遍 QP 落到 −3…6，`ratio` 失真 |
| K | RC 的 `intraPeriod` 被钳到 ≥ `GOPSize` | `:98` | 15 fps 下实际 `-ip 16` 而 RC 内为 32；`-ip 8/16/32` 实测 75.90/67.51/53.60 kbps，RC 的 IGOP 封顶（`:1266`）一直按 32 计算 |
| L | 测试只校验字节一致性，不校验码率；无 `MaxBitrate` 用例 | `cmake/modules/vvencTests.cmake:93-99` | `ctest -R rc2p` 10/10 通过，但上述边界零覆盖 |

### 4.2 建议（未实现）

1. **EOS 窗口按真实长度缩放**：`flushPOC = lastPoc - min(N, max(GOPSize, intraPeriod))`，并把 `32` 换成具名常量。优先级最高，直接消除 §3.1 的台阶。
2. **让 EOS 只在 `flush` 时激活**：两遍模式在 `processFirstPassData(false)` 阶段就知道了完整序列长度，没有必要预设末段。
3. **I 帧桶按片段长度决定是否独立**：`N < GOPSize` 时并入首 GOP 桶。
4. **解耦 `dLimit` 与 `MaxBitrate`**，并保证 `d` 关于 `maxGopRate` 单调。修复 §3.2 的"收紧上限抬高码率"。
5. **预封顶再分配改为按比例**（而不是 `savedBits /= 未封顶桶数`），并在每次跨 GOP 时重算 `scale`。
6. **把 RC 内 `intraPeriod` 换成编码器实际生效值**（`m_IntraPeriod` 与 `GOPSize` 取 min，或直接用 `GOPCfg` 的实际 I 帧间距），修复 §1.2/表 4.1-K 的公式错配。
7. **`rcQP` 加下界地板**，并在 `getBaseQP()` 中检测第一遍 QP 触底。
8. **补测试**：至少覆盖 `N ∈ {31,32,33}`、`N < GOPSize`、`MaxBitrate` 三档、以及"目标码率远低于可达码率"四种情形，断言**码率落在目标 ±X%** 而不只是字节一致。当前 47 个 ctest 用例中没有任何一个做码率断言（`ctest -N` 共 47 项）。

---

## 附录：复现命令与素材

素材（均为秒级编码，未做长时编码）：

```bash
# 从仓库自带 15 帧素材派生的测试素材（80x44, yuv420p, 15fps）
#   /tmp/rcdata/noise300.yuv            300 帧随机噪声
#   /tmp/rcdata/c300.yuv                300 帧 = 自带素材循环
#   /tmp/rcdata/cut32.yuv               64 帧，poc 32 处硬切（后 32 帧取反）
#   /tmp/rcdata/nocut64.yuv             64 帧，无切
#   /tmp/rcdata/st128.yuv               32 帧静止 + 96 帧噪声
```

构建与运行：

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release && cmake --build build -j4

# 场景 A：末段窗口台阶
./bin/release-static/vvencFFapp -i /tmp/rcdata/noise300.yuv -s 80x44 -fr 15 \
  -f 32 --NumPasses 2 --TargetBitrate 100000 -ip 32 -b /tmp/A32.vvc -v 5
./bin/release-static/vvencFFapp -i /tmp/rcdata/noise300.yuv -s 80x44 -fr 15 \
  -f 33 --NumPasses 2 --TargetBitrate 100000 -ip 32 -b /tmp/A33.vvc -v 5

# 场景 B：MaxBitrate 收紧
./bin/release-static/vvencFFapp -i /tmp/rcdata/noise300.yuv -s 80x44 -fr 15 \
  -f 300 --NumPasses 2 --TargetBitrate 100000 --MaxBitrate -24 -ip 32 -b /tmp/B.vvc -v 5

# 额度不足
./bin/release-static/vvencFFapp -i /tmp/rcdata/noise300.yuv -s 80x44 -fr 15 \
  -f 96 --NumPasses 2 --TargetBitrate 5000 -ip 32 -b /tmp/D.vvc -v 5

# 仓库自带两遍用例（CI 等价）
ctest --test-dir build -R rc2p --output-on-failure     # 10/10 通过
```

插桩说明：§1/§2/§3 中标为"插桩观察"的中间量（`flushPOC`、`isEndOfSequence`、`dLimit`、`tmpTargetBits`、`estimatedBitUsage`、`isNewScene`）由临时加入 `RateCtrl.cpp` 的 `fprintf` 探针取得，探针已通过 `git checkout -- source/Lib/EncoderLib/RateCtrl.cpp` 完全撤销，仓库无残留改动。
