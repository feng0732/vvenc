# VVC 自适应环路滤波（ALF）参数的产生、传递与参考画面生效

本文基于仓库当前 HEAD（`58cbc54`，`vvenc 1.15.0-dev`）的源码和实测码流，回答三个问题：

1. ALF 参数什么时候被算出、被写进哪里（APS / slice header / CTU 层）；
2. 参数如何进入重建参考画面（即被后续帧用作参考的滤波后重建）；
3. 参数集复用（temporal prediction / `ALFTempPred`）与"参考画面生效"之间是什么关系。

文中所有结论都在源码中有明确落点，并在第 6 节给出可复现的观测。

---

## 1. 参数产生：三个层次

### 1.1 每个 picture 的 ALF 处理是四段流水

ALF 不是在编码完整个 picture 之后一次性做的，而是在 `EncSlice` 的 CTU 流水线里按阶段推进（`source/Lib/EncoderLib/EncSlice.cpp`）：

| 阶段 | 位置 | 做什么 | 关键前提 |
|---|---|---|---|
| `ALF_GET_STATISTICS` | `EncSlice.cpp:1220` | 对重建（deblock+SAO 后、ALF 前）样本累计协方差统计 | 等待右/下/右下 CTU 完成 SAO（`checkCtuTaskNbRgt/Bot/BotRgt`，`:1222-1224`）|
| `ALF_DERIVE_FILTER` | `EncSlice.cpp:1254` | `initDerivation` + `deriveFilter` + `reconstructCoeffAPSs` | 常规模式下 `deriveFilterCtu` 是整帧最后一个 CTU，所以必须等所有行的统计（`:1258-1272`）|
| `ALF_RECONSTRUCT` | `EncSlice.cpp:1310` | 用选定的滤波器重建（覆盖写回 `cs.getRecoBuf()`）| 必须等 `deriveFilterCtu` 进入 `ALF_RECONSTRUCT`（`:1314-1315`）|
| `CCALF_*` | `EncSlice.cpp:1356` 起 | CC-ALF 统计、训练、重建 | luma 统计读的是 ALF 前的拷贝 `m_tempBuf`（`EncAdaptiveLoopFilter.cpp:6421`）|

统计阶段的输入是 `m_tempBuf`：`copyCTUforALF`（`EncAdaptiveLoopFilter.cpp:1930`）在 SAO 之后把含 padding 的 ALF 前重建拷进去。也就是说 **ALF 统计永远基于"本帧 ALF 前重建"，而不是任何参考帧的滤波结果**。

### 1.2 帧级参数：`AlfParam` 与 chroma alternatives

`EncAdaptiveLoopFilter::deriveFilter`（`EncAdaptiveLoopFilter.cpp:1967`）先聚合 ASU 协方差到 `m_alfCovarianceFrame`，然后：

- `alfParam.newFilterFlag[CH_L] = true`、`alfParam.newFilterFlag[CH_C] = isChromaEnabled(...)`、`cs.slice->numAps = 1`（`EncAdaptiveLoopFilter.cpp:2043-2046`），先做一轮"只允许新滤波器"的 RD 成本估算；
- `alfEncoder` 为亮度求 `numLumaFilters`、25 个 class 到 filter 的映射（`filterCoeffDeltaIdx`）、系数与 clipping（`:2632`）；
- chroma 在 `alfEncoderCtb` 的后半段处理（`:5358` 起），每个 chroma APS 可以带 `numAlternativesChroma` 组滤波器（`VVENC_MAX_NUM_ALF_ALTERNATIVES_CHROMA = 8`），CTU 通过 `alf_ctb_alternative_idx` 选择（`CABACWriter::codeAlfCtuAlternative`，`CABACWriter.cpp:3156`）。

最终只有一片 APS 和 slice header 里的几个 id 会进入码流，帧内细节见下。

### 1.3 CTU 级参数：使能标志 + 滤波器集索引 + chroma alt

`alfEncoderCtb`（`EncAdaptiveLoopFilter.cpp:5035`）为每个 ASU/CTU 决定：

- luma：`alf_ctb_flag`（`CABACWriter::codeAlfCtuEnabledFlag`，`CABACWriter.cpp:2983`）与滤波器集索引（`codeAlfCtuFilterIndex`，`:3090`）。可用滤波器集 = 16 个固定集（`NUM_FIXED_FILTER_SETS = 16`，`CommonDef.h:263`）+ 本帧新算的集 + slice header 引用的 temporal APS 集，集总数见 `CABACWriter.cpp:3100-3101`（`numAps + NUM_FIXED_FILTER_SETS`）。索引编码见 `:3102-3123`：先一位 `useTemporalFilt`，再定长/截断二值码。
- chroma：`alf_ctb_flag` 与 alternative 索引（`CABACWriter::codeAlfCtuAlternative`，`:3156`）。

所以"参数"实际分三级：**帧级系数集（APS 里）→ 本帧引用哪些 APS（slice header）→ 每个 CTU 是否开、用哪一套（CABAC）**。

---

## 2. 参数如何写进码流，以及在哪一层被"激活"

### 2.1 APS 的生成与写出的时机是解耦的

关键点：**APS NAL 是在 `xWriteParameterSets` 里按 `changedFlag` 懒写出的，不是在 ALF 处理时写出**。

- `initDerivation`（`EncAdaptiveLoopFilter.cpp:2365`）把 `m_apsMap` 指向 `slice.pic->picApsMap`、`m_apsIdStart = picApsMap.getApsIdStart()`（`:2377-2378`）；新 APS id 取 `m_apsIdStart - 1`（`:2413`），下溢时回绕到 7（`:2418-2421`）。
- 一旦决定要发"新滤波器"，代码设置 `m_apsMap->setChangedFlag(...)` 并 `m_apsIdStart = newApsId`（luma：`:5347-5348`；chroma：`:5568-5569`；CC-ALF：`:1684`、`:1721`）。
- 写出发生在 `EncGOP::xWriteParameterSets`（`EncGOP.cpp:2558`）：

```cpp
if ( sps.alfEnabled && (slice->alfEnabled[COMP_Y] || slice->ccAlfCbEnabled || slice->ccAlfCrEnabled ))
{
  for ( int apsId = 0; apsId < ALF_CTB_MAX_NUM_APS; apsId++ )   // 8 个 id，从 0 升序
  {
    APS* aps = apsMap.getPS( apsMapIdx );
    bool writeAps = aps && apsMap.getChangedFlag( apsMapIdx );
    if ( writeAps ) { ... xWriteAPS(...); apsMap.clearChangedFlag( apsMapIdx ); }
  }
}
```
（`EncGOP.cpp:2584-2601`）

由此得到三个可观察的行为：

1. **APS 在前、slice 在后**：`xWritePicture` 先 `xWriteParameterSets` 再 `xWritePictureSlices`（`EncGOP.cpp:2546-2548`）。同一 AU 里 APS NAL 位于 slice NAL 之前。
2. **参数"造出"与"写出"是两个不同时刻**。置 `changedFlag` 发生在 ALF 处理中（`compressPicture` 内），而真正写 NAL 发生在 AU 输出时（`xProcessPictures` 的 output 阶段调用 `xWritePicture`）。两者之间隔着 `finalizeReference`（把 APS 拷进全局列表，`EncPicture.cpp:139-158`）与 `finalizePicture`（写 slice data，`EncPicture.cpp:162-180`）。因此**一个后续 picture 可以在 APS NAL 尚未写出时就已经引用它**；正确性依赖两点：(a) AU 输出顺序严格等于 coding order（`m_gopEncListOutput` 是 FIFO，`EncGOP.cpp:540-558`）；(b) 被引用的 APS id 一定来自 coding order 上更早的 picture。这两点目前成立，但都是隐式不变量，没有任何断言保护。
3. **写出条件是 slice 级开关，而 flag 的设置位置在更早的 RD 分支里**。`xWriteParameterSets` 的外层条件是 `alfEnabled[Y] || ccAlfCbEnabled || ccAlfCrEnabled`（`EncGOP.cpp:2584`），而新 APS 的 `setChangedFlag` 只出现在"ALF 保持开启"的分支内（luma `:5347`、chroma `:5568`、CC-ALF `:1684/:1721`）。也就是说，"造了新 APS 却因外层条件为假而不写出、flag 残留到全局列表"这类问题**当前不可达**，但它是靠分支布局而非显式约束保证的 —— 只要将来有人把 `setChangedFlag` 移到 RD 决策之前（例如为两遍决策预写 APS），残留 flag 就会随 `finalizeReference` 进入全局 APS 列表，并在后续帧的 AU 里被写出。同理，CC-ALF 打开而 luma ALF 关闭时，该帧会写出 ALF APS 但 `sh_alf_enabled_flag = 0`；这本身合法（APS 是独立 NAL），但写出的判据与 slice header 语义已不再一一对应。

### 2.2 slice header 里到底写了什么

`HLSWriter::codeSliceHeader`（`VLCWriter.cpp:1719-1762`）：

```
sh_alf_enabled_flag                      // = slice->alfEnabled[COMP_Y]
if (enabled) {
  sh_num_alf_aps_ids_luma  (3 bit)       // = slice->numAps
  sh_alf_aps_id_luma[i]    (3 bit each)  // = slice->lumaApsId[i]
  sh_alf_cb_enabled_flag / sh_alf_cr_enabled_flag
  sh_alf_aps_id_chroma     (3 bit)       // 只要 Cb||Cr 打开就写
  sh_cc_alf_cb_enabled_flag → sh_cc_alf_cb_aps_id
  sh_cc_alf_cr_enabled_flag → sh_cc_alf_cr_aps_id
}
```

注意 `sh_alf_enabled_flag` 为 0 时，**chroma 和 CC-ALF 的所有信息都不写**。这与 `CABACWriter::coding_tree_unit` 的 CTU 层 gating 一致：`if (cs.slice->alfEnabled[compIdx])`（`CABACWriter.cpp:170-184`）。但 **CC-ALF 的 CTU 层控制 idc 用的是另一套条件** `cs.slice->ccAlfFilterParam.ccAlfFilterEnabled[compIdx-1]`（`CABACWriter.cpp:187`），而 `alfEnabled` 又是 `deriveCcAlfFilter` 的前置（`EncAdaptiveLoopFilter.cpp:6119-6123` 中 `if (!cs.slice->alfEnabled[COMP_Y]) { ...Enabled=false; return; }`）。两处状态来源不同、却没在写码前统一断言，是后续改动的风险点。

### 2.3 `sh_num_alf_aps_ids_luma` 的语义：它只是"引用列表"，不是"更新"

这是理解后文的关键。`numAps` 与 `lumaApsId` 表示"本帧 slice 允许使用的 temporal 滤波器集"，可用的滤波器**集索引**是 `NUM_FIXED_FILTER_SETS + i`，i 是该列表中的位置（`CABACWriter.cpp:3098-3101`；解码端 `CommonLib/AdaptiveLoopFilter.cpp:278-297` 的 `reconstructCoeffAPSs` 按同样顺序重建 `m_coeffApsLuma[i]`）。

因此：

- `numAps` 可以为 0，而本帧仍然使用 ALF（因为可以使用 16 个固定集，见 `codeAlfCtuFilterIndex` 的 `numAvailableFiltSets == NUM_FIXED_FILTER_SETS` 分支，`CABACWriter.cpp:3121-3125`）。实测就是这样（6.2 表里大量 `nAPS=0` 且 `en=1`）。
- 若本帧用了"新算的 **luma** 滤波器"，新 APS 的 id 一定会出现在 `lumaApsId` 里（`bestApsIds` 在 `:5285-5299` 组装，`cs.slice->setAlfApsIds(bestApsIds)` 在 `:5327-5328`）。**注意反过来不成立**：本帧可以只写一个新 **chroma** APS 而 `numAps = 0`（6.2 的 POC 15 即此情形），因为 `chromaApsId` 不受 `numAps` 约束。

---

## 3. 参考画面生效：ALF 结果何时可以被别的帧读到

### 3.1 参考帧读到的是"ALF 后重建"

`Picture` 的 `PIC_RECONSTRUCTION` buffer 由 `EncPicture::compressPicture` 期间创建（`Picture.cpp:325-332`），`cs.m_reco` 通过 `createFromBuf` 指向同一块内存（`CodingStructure.cpp:646`）。`reconstructCTU` 直接写 `cs.getRecoBufRef()`（取引用在 `EncAdaptiveLoopFilter.cpp:2144`；luma 写入在 `:2260`（无虚拟边界路径），chroma 在 `:2273`），`applyCcAlfFilterCTU` 写 `cs.getRecoBuf().get(compID)`（`:7107`）。

而 inter prediction 读的是 `refPic->getRecoBuf(...)` / `getRecoBufPtr(...)`（`InterPrediction.cpp:826-827`、`:1131`、`:1729`）。**所以参考帧送进运动补偿的，就是已经过 ALF（及 CC-ALF）的像素**。这就是"滤镜参数必须在该帧成为参考之前定稿"的根本原因：不是一个缓存时序问题，而是同一块内存的读写顺序问题。

### 3.2 可用性标志的两段式提交

`Picture::isReconstructed` 的注释写得很直白（`Picture.h:229`）：

```cpp
std::atomic_bool isReconstructed;  // usable as a reference: reconstruction, motion and ALF APSs final
std::atomic_bool isEncoded;        // finalizePicture done too: slice data written, picture encoder free
```

- `EncPicture::finalizeReference`（`EncPicture.cpp:114`）：`pic.picApsMap.setApsIdStart(m_ALF.getApsIdStart())`、把 CC-ALF 参数与控制 idc 抄回 slice（`:128-130`）、把 ALF APS 拷进 `pic.picApsGlobal->apsMap`（`:139-158`）。之后才置 `isReconstructed`（`EncGOP.cpp:735`、`:760`）。
- `EncPicture::finalizePicture`（`:162`）：写 slice data（`xWriteSliceData` → `encodeSliceData`）、算失真、释放中间数据，之后置 `isEncoded`。

这条分界线正是"参数进入参考"的边界：**在 `isReconstructed` 之后，该帧的重建像素与它写出的 ALF APS 都已经定稿**。

### 3.3 谁在等这个标志

`EncGOP::xEncodePicture` 开始处：

```cpp
if( m_pcEncCfg->m_alf && m_pcEncCfg->m_alfTempPred )
  xSyncAlfAps( *pic );          // EncGOP.cpp:693-697
```

`xSyncAlfAps`（`EncGOP.cpp:611`）从 `pic.refApsGlobal` 把上一帧的 APS 表整体拷进当前帧的 `picApsMap`，并把 `apsIdStart` 继承过来（`:667-676`）。这个 `refApsGlobal` 指向哪里，由 `xSetupPicAps`（`EncGOP.cpp:1484`）决定：

- 单线程：取全局列表里**紧邻的前一个**（coding order）picture（`EncGOP.cpp:1538-1544`）。
- MT：取更早的、`tid` 严格小于当前 layer 的那一个（`:1528-1537`）；intra 帧直接不设参考（`:1512-1516`）。
- 就绪性由 `isReady` 判定：`!cand->refApsGlobal || cand->refApsGlobal->initalized`（`EncGOP.cpp:474`），而 `initalized` 在 `finalizeReference` 之后的 finish task 里置位（`:737`、`:762`）。

所以 `ALFTempPred` 是一条显式的"参数跨帧继承链"，且这条链的推进被 `isReconstructed` 严格串起来 —— 这是本次分析中最干净的一处设计。

---

## 4. 参数集复用（temporal prediction）与参考画面的关系

### 4.1 复用逻辑本身

`getAvaiApsIdsLuma`（`EncAdaptiveLoopFilter.cpp:4669`）：

```cpp
int apsIdChecked = 0, curApsId = m_apsIdStart;
if( curApsId < ALF_CTB_MAX_NUM_APS && !slice.isIntra() && !slice.pendingRasInit && !slice.isIDRorBLA() )
  while (apsIdChecked < ALF_CTB_MAX_NUM_APS) {
    APS* curAPS = slice.alfAps[curApsId];
    if (curAPS && curAPS->layerId <= slice.pic->layerId && curAPS->temporalId <= slice.TLayer) {
      if (curAPS->alfParam.newFilterFlag[CH_L]) result.push_back(curApsId);
      if (curAPS->alfParam.newFilterFlag[CH_C]) m_apsIdsChroma.push_back(curApsId);
    }
    apsIdChecked++; curApsId = (curApsId + 1) % ALF_CTB_MAX_NUM_APS;
  }
```

三点含义：

1. **候选集是"上一帧之后累积的 APS 表"，按环形 id 顺序遍历**，不是按 POC 排序，也不是按 RD 质量排序。
2. **过滤条件只有 `temporalId <= TLayer`（temporal 层限制）与 `layerId`**。没有"滤波器是否仍然适用"的判据；是否使用，交给 `alfEncoderCtb` 的 RD 循环（`:5064-5306`）去试 `numTemporalAps = 0..apsIds.size()` × `useNewFilter ∈ {0,1}` 的组合。
3. **`m_apsIdStart` 是复用的"水位"**：新 APS 总是占 `m_apsIdStart - 1`，因此最新的滤波器 id 与最旧的 id 在环形空间上相邻。`ALF_CTB_MAX_NUM_APS = 8`（`CommonDef.h:262`）意味着最多 8 套 luma 滤波器同时在环上。

`m_apsIdsChroma` 在同处填充（`EncAdaptiveLoopFilter.cpp:4688-4690`），但**在仓库中没有任何读取点** —— 唯一出现是 `initDerivation` 的清空（`:2375`）与这里的 push。chroma 的复用实际走另一条路径 `alfEncoderCtb`（`:5035`）里的逐 id 扫描：

```cpp
else if (m_encCfg->m_alfTempPred && curAPS && curAPS->temporalId <= maxTID && curAPS->alfParam.newFilterFlag[CH_C])
  m_alfParamTemp = curAPS->alfParam;        // EncAdaptiveLoopFilter.cpp:5414
```
（`:5398-5420`）

也就是说 luma 的候选集是**一次算好**的 `apsIds` 列表，chroma 的候选集是**在 `alfEncoderCtb` 内现场**扫 `0..ALF_CTB_MAX_NUM_APS-1` 并按 `curAPS->layerId != layerId` 与 `temporalId` 过滤。`m_apsIdsChroma` 是这段历史的残留物（write-only 成员），可以删掉。CC-ALF 的复用另有 `getAvailableCcAlfApsIds`（`:6019`），条件同上并额外要求 `curAPS->ccAlfParam.newCcAlfFilter[compID-1]`。

### 4.2 "复用"与"参考画面"的耦合点

复用链依赖 `picApsMap` 的三个拷贝环节，缺一不可：

```
帧 N 编码结束
  └─ finalizeReference: picApsMap ──copy──▶ picN.picApsGlobal->apsMap      (EncPicture.cpp:141-158)
帧 N+1 开始 xEncodePicture
  └─ xSyncAlfAps: refAps->apsMap ──copy──▶ pic{N+1}.picApsMap              (EncGOP.cpp:629-676)
     (refAps 由 xSetupPicAps 选定，可 MT 时跳层)
帧 N+1 的 ALF: m_apsMap = &picApsMap                                      (EncAdaptiveLoopFilter.cpp:2378)
```

三个直接后果：

- **复用链的"参考"是 APS 表，不是重建像素**。参数可以在一个根本没有使用 ALF 的帧上继续传递：`xSyncAlfAps` 不看 `alfEnabled`，`finalizeReference` 里 APS 拷贝也不看（`EncPicture.cpp:139-158` 的 `if (pic.picApsGlobal)` 与 ALF 是否启用无关）。实测证据（6.2）：POC 15 写出 chroma APS id 6 并立即被 POC 7 以 `sh_alf_aps_id_chroma=6` 引用；随后 POC 3、1、0、2、5、4、6、11、9、8、10、13、12、14 **全部 `sh_alf_enabled_flag = 0`**，而 **POC 23 仍以 `sh_alf_aps_id_chroma=6` 引用了同一个 APS**。即这条参数链穿过了 14 个"ALF 关闭"的 picture。
- **MT 帧并行会切断某些链**：`xSetupPicAps` 在 MT 且 intra 时直接 return（`EncGOP.cpp:1512-1516`），`xSyncAlfAps` 同样（`:616-620`）。此外 MT 且 `TLayer == 0` 时，会限制传播的 TID-0 APS 数量，只搬运"poc 最大"的若干个（`:632-660`）。这条策略在 `maxTLayers > ALF_CTB_MAX_NUM_APS` 时会直接 CHECK 失败（`:635`）。
- **`apsIdStart` 的继承跨越了帧**：`xSyncAlfAps` 末尾 `dst.setApsIdStart(src.getApsIdStart())`（`:676`），`finalizeReference` 里同样（`EncPicture.cpp:157`）。所以"下一个新 id"由参数链而非 POC 决定。

### 4.3 与 `ALFSpeed` 的交互

`isSkipAlfForFrame`（`EncAdaptiveLoopFilter.cpp:2508`）：

```cpp
if( !sps->alfEnabled || ( sps->maxTLayers > 1 && sps->maxTLayers - m_encCfg->m_alfSpeed <= pic.TLayer ) )
  return true;
```

`ALFSpeed = 1` 时最高 temporal layer 整帧跳过 ALF。跳过发生在 `getStatisticsCTU`（`EncAdaptiveLoopFilter.cpp:1738`）、`deriveFilter`（`:1971`）、`reconstructCTU_MT`（`:2077`）、`initEncProcess`（`:2306`）、`initDerivation`（`:2367`）、`selectFilterForCTU`（`:4724`）六个入口的同一判断。这意味着：

- 被跳过的帧**既不改动重建，也不新增 APS**。此时 `initEncProcess`（由 `EncSlice::initPic` 在 `EncSlice.cpp:840` 调用）提前返回，不会把 `sps->alfEnabled` 写进 slice；随后 `EncPicture::xInitPicEncoder` 把 `pic.slices[s]->alfEnabled[COMP_Y]` 置 false（`EncPicture.cpp:266-271`），而 `alfEnabled[COMP_Cb/Cr]` 保持 slice 构造时的 false（`Slice.cpp:118`）。结果是**跳过的帧在码流里 `sh_alf_enabled_flag = 0`**，并且不贡献新参数。注意 `xInitPicEncoder` 只清 COMP_Y，不清 Cb/Cr —— 当前没问题（三者初值不同步的风险被 `sh_alf_enabled_flag` 为 0 时"chroma 信息不写"掩盖），但值得补一次三者的统一清零。

这一点实测非常清楚（6.2 中 `ALFSpeed=1` 的 `randomaccess_fast` 里，TID ≥ 5 的帧 ALF 全关）。

---

## 5. 讨论：滤波开关、参数更新滞后、分量差异、参考切换

### 5.1 滤波开关分五级，彼此不共享状态源

| 级别 | 位置 | 判定依据 |
|---|---|---|
| SPS | `EncGOP.cpp:1134-1135` | `m_alf` / `m_ccalf` |
| picture（整帧跳过）| `EncAdaptiveLoopFilter.cpp:2508` | `maxTLayers - m_alfSpeed <= TLayer` |
| slice 级 ALF/CC-ALF | `EncPicture.cpp:266-271` 初始化；`alfEncoderCtb` 改写（`EncAdaptiveLoopFilter.cpp:5316-5335`）| RD 比较 `costOff <= costMin` |
| slice 级 temporal APS 列表 | `EncAdaptiveLoopFilter.cpp:5326-5328` | `bestApsIds`（`:5284-5299`）|
| CTU 级 | `alfEncoderCtb` / `selectFilterForCTU` | RD（含 `alf_ctb_flag` 与索引码率）|

值得注意的两处不一致：

1. **`costOff <= costMin` 用了 `<=` 并且在 IFP 下被完全禁用**（`EncAdaptiveLoopFilter.cpp:5315-5316`）：
   ```cpp
   // in case of IFP lines synchro, we always trying to use ALF (with final decision at CTU level)
   if (costOff <= costMin && !m_encCfg->m_ifpLines)
   ```
   即 IFP（`m_ifpLines > 0`）时 slice 级 ALF **永远不会**关，只能靠 CTU 标志逐块关。这与 `initEncProcess` 的注释一致（`:2311-2312`），是"行同步流水要求参数在整帧内固定"这一约束的直接体现 —— 因为 `ALF_DERIVE_FILTER` 在 IFP 模式下只等到 `m_alfDeriveCtu`（`EncSlice.cpp:309-312`，`ifpLines` 向上取整到一个 ASU 边界再加一个 ASU 高），此时整帧 luma 统计尚未收集完，滤波器是**用部分数据**推导的（`changelog.txt:202`："allow ALF derivation from partial data for FPPLinesSynchro"）。既然参数是用不完整数据算的，"整帧关闭"的 RD 比较就不可靠，于是干脆不做。
2. **`slice.alfEnabled[COMP_Cb/Cr]` 与 `m_ccAlfFilterParam.ccAlfFilterEnabled[]` 来源不同**，前者由 chroma RD 决定（`:5516-5520`），后者由 CC-ALF 训练决定（`:6335`）。写码时 `coding_tree_unit` 用前者 gate ALF CTU 语法（`CABACWriter.cpp:170-187`）、用后者 gate CC-ALF 控制 idc 语法（`:189-199`）。二者不一致时（例如 chroma ALF 关而 CC-ALF 训练后判定"至少一个 CTU 需要滤波"）会产生 `ccAlfFilterParam` 需要 APS 但没有 chroma APS id 的局面 —— `deriveCcAlfFilter` 里默认把 CC-ALF 的新 APS id 复用 luma 的 `newApsId`（`:5311-5312`）以规避，但该赋值发生在 luma RD 结束、chroma RD 之前，属于隐式耦合。

### 5.2 参数更新滞后（lag）

"滞后"在两个不同意义上都存在：

**(a) 帧内滞后。** `ALF_DERIVE_FILTER` 必须先于 `ALF_RECONSTRUCT`。常规模式下滤波器在最后一个 CTU 才定稿，因此**整个 picture 的 CTU 都必须等到最后一行的统计完成**才能开始重建（`EncSlice.cpp:1258-1272`）。这直接决定了 ALF 无法在帧内做"逐行自适应参数"——CTU 只能选（16 固定集 + 已定稿 temporal APS + 新集），不能各自算。IFP 模式部分放宽：`m_alfDeriveCtu` 取 `ifpLines` 之后的一个 ASU 边界（`:309-311`），先定稿一套参数供后续行选择，代价是 slice 级不能关 ALF（见 5.1）。

**(b) 帧间滞后。** 新算的滤波器要等下一帧（或更晚）才能被复用，因为 APS 在本帧才写出，而本帧 slice header 引用的是本帧的 `lumaApsId`。一个容易误读的点是"新 APS 写出"与"`numAps`"的关系，实际是**互推的**：`bestApsIds.resize(numFilterSet - NUM_FIXED_FILTER_SETS)` 而 `numFilterSet = NUM_FIXED_FILTER_SETS + numTemporalAps + useNewFilter`（`EncAdaptiveLoopFilter.cpp:5099-5100`、`:5285-5299`），所以 `newFilterFlag[CH_L] == useNewFilter` 为真时 `bestApsIds` 至少含 `newApsId` 一项（`:5290`），因此 **`sh_num_alf_aps_ids_luma = 0` 的帧一定没有写新的 luma APS**。实测与此完全一致：6.2 表里 POC 27/29/30/32 都是 `numAps=0`，它们的 AU 中也确实没有任何 APS（"AU 里的 APS"列为空）。反过来，真正"为将来投资"的 APS 一定出现在 `numAps >= 1` 的帧上（例：POC 31 的 luma APS 7）。

真正的滞后在 chroma：`chromaApsId` 的有效值可以在 `numAps = 0` 时存在（POC 15 的 APS 6），因为 chroma 的 id 与 luma 的 `numAps` 是**两个独立的数**，`numAps` 只描述 luma 的列表长度（`VLCWriter.cpp:1726-1728`）。

**(c) 一个隐藏的滞后：`isSkipAlfForFrame` 的帧既不更新参数也不更新重建。** 于是当 `ALFSpeed=1` 跳过了最高 TID 的若干帧后，复用的是**更早、更"旧"的滤波器**。在内容快速变化时这是可见的质量损失。

### 5.3 分量差异

| | luma | chroma | CC-ALF |
|---|---|---|---|
| 滤波器 | 7x7，25 classes，最多 `MAX_NUM_ALF_LUMA_COEFF` 系数/类 | 5x5，1 class，最多 8 组 alternatives | 3x4/5x5 跨分量，最多 4 个 filter |
| APS 中的字段 | `newFilterFlag[CH_L]` | `newFilterFlag[CH_C]` + `numAlternativesChroma` | `newCcAlfFilter[Cb/Cr]` |
| slice header | `sh_num_alf_aps_ids_luma`（可多个）| `sh_alf_aps_id_chroma`（单值）| `sh_cc_alf_cb_aps_id` / `_cr_aps_id` |
| CTU 语法 | 1 个 enable + 1 个 filter idx | 1 个 enable + 1 个 alt idx | 1 个 control idc（0..filterCount）|
| 复用粒度 | id 环，8 个 | 单 id | 单 id / 分量 |

关键差异：

- **luma 可以同时引用多个 APS**（`numAps` 最多 8），CTU 在这 8 个里选；**chroma 只能引用一个**（`sh_alf_aps_id_chroma` 单值），但一个 APS 内可以有多个 alternatives 供 CTU 选。这是设计上的取舍：luma 的 25 类映射使"多个滤波器集"划算，chroma 用 alternatives 更省 slice header。
- **CC-ALF 的 APS 挂在 luma/chroma 共用的 APS id 上**：`xSetupCcAlfAPS` 使用 `cs.slice->ccAlfCbApsId`（`:1659-1691`），而该 id 在 luma RD 结束后被无条件设成新的 luma APS id（`:5311-5312`），在 CC-ALF 复用命中时又被改成旧 id（`:6386-6399`）。所以**同一个 APS id 可以同时承载 luma 系数、chroma 系数、CC-ALF 系数**，三者的 `newFilterFlag` / `newCcAlfFilter` 相互独立。`deriveCcAlfFilter` 开头会主动把"本帧已 changed 的 luma/chroma APS"里的 `newCcAlfFilter` 清零（`:6085-6103`），避免同一 APS 在同一次写出中既报新 luma 又报新 CC-ALF —— 这是对上述"共 id"设计的补丁式约束，也意味着**CC-ALF 启用的那一帧一定同时写了该 id 上的新 luma 或新 chroma 系数**（否则它依赖的 APS 未变、`newCcAlfFilter` 会被清零，CC-ALF 参数无处存放）。
- **chroma 的 `alfEnabled` 与 CC-ALF 的 `ccAlfFilterEnabled` 是两套状态**。`deriveCcAlfFilter` 在 luma ALF 关闭时早退并把 `m_ccAlfFilterParam.ccAlfFilterEnabled[]` 置 false（`:6119-6123`）；`alfEncoderCtb` 的 luma 关闭路径会 `memset(cs.slice->alfEnabled, 0, ...)` 并直接 return（`:5318-5324`）。两条路径保证了"luma 关 ⇒ chroma ALF 与 CC-ALF 都关"，但这是由两个不同函数、在两个不同阶段各自维护的，没有单点约束。**CC-ALF 因此不能在没有 luma ALF 的帧上工作**，尽管码流语法上 `sh_cc_alf_*` 只受 `sh_alf_enabled_flag` 管辖。
- **`sh_alf_aps_id_chroma` 是单值，没有"numChromaAps"**；编码端 `chromaApsId = -1` 表示"本轮没有 chroma APS"（`:5535`）。`CABACWriter.cpp:181` 直接下标 `cs.slice->alfAps[cs.slice->chromaApsId]`，安全性完全依赖"`alfEnabled[Cb]||alfEnabled[Cr]` 为真 ⇒ `chromaApsId >= 0`"这一不变量；而三者在 `:5528-5536` 的**同一个分支**里成对赋值（`:5518-5519` 设 id 与 flag，`:5532-5535` 一起清），所以当前是安全的。但这是纯隐式契约：任何在别处单独修改 `alfEnabled[COMP_Cb]`（例如 5.1 提到的 IFP 路径或多 slice 场景）的改动都会让这里越界。

### 5.4 参考画面切换

参考画面的选择（RPL）与 APS 的传递链是**两条独立的链**：

- `refApsGlobal` 由 `xSetupPicAps` 按 **coding order / temporal layer** 选（`EncGOP.cpp:1528-1544`）；
- RPL 由 GOP 配置（`cfg/gop32.cfg`）决定。

二者不保证一致。例如 POC 3 的 L0 是 `{7,15}`，而它的 APS 链来自 coding order 上紧邻的 POC 7（POC 7 从 POC 15 继承）。因此"某个参考帧的重建用了滤波器 A"与"我能复用滤波器 A"是两件事：**参考帧的重建里已经烘进了 A，而我是否能取到 A，取决于 APS 链上 A 是否还在环里**。这正是 VVC 中 ALF 与帧间预测解耦的设计：解码端也不需要"参考帧用了什么滤波器"，只需要"我引用的 APS 内容是什么"。

MT 帧并行下这一点被进一步削弱：`xSetupPicAps` 在 `mtPicParallel && isIntra` 时直接 return（`:1512-1516`），IRAP 帧成为 APS 链的断点，其后的帧从**更早的非 intra 帧**继承（`find_if` 反向查找，`:1528-1537`）。这是有意的：IRAP 之前的内容对随机接入不可用，参数链也一并切断。但注意 `xSyncAlfAps` 在 `mtPicParallel && slice.isIntra()` 时也 return（`:616-620`），而 `initDerivation` 里对 IDR/BLA 的清空是**无条件**的（`:2382-2398`），两处语义并不完全对齐：前者按线程模式，后者按 NAL 类型。

---

## 6. 可观察的重建与码流结果

以下实验均在本仓库 HEAD 上构建（`cmake -DVVENC_ENABLE_TRACING=ON`），使用 `--tracerule=D_HEADER` 导出 slice header / APS 语法，`D_MISC` 导出 ALF 推导顺序。

### 6.1 测试素材

- `test/data/RTn23_80x44p15_f15.yuv`（仓库自带，80x44，15 帧）
- 合成序列 `synth352_48.yuv`（352x240，48 帧，正弦+噪声，chroma 与 luma 有弱相关）
- 合成序列 `ccsrc.yuv`（352x240，40 帧，块状图案 + chroma 与 luma 强相关，用于触发 CC-ALF）
- `testsrc2`（ffmpeg 生成，352x240，64 帧）

### 6.2 典型 APS 时间线（`randomaccess_medium`, `gop32`, Threads=1, 48 帧）

`hdr48.txt` 的解析结果。`编码序号` 是该 picture 在 **coding order**（= AU 输出顺序）中的位置，最后一列是它所在 AU 携带的 APS：

| 编码序号 | POC | sh_alf_enabled | nAPS | luma APS | Cb/Cr | chroma APS | 该 POC 的 AU 里带的 APS (id: luma/chroma) |
|---:|---:|---:|---:|---|---|---|---|
| 0 | 31 | 1 | 1 | [7] | 0/0 | – | 7: 1/0 |
| 1 | 15 | 1 | 0 | [] | 1/1 | 6 | 6: 0/1 |
| 2 | 7 | 1 | 0 | [] | 1/1 | 6 | – |
| 3 | 3 | 1 | 0 | [] | 0/0 | – | – |
| 4–16 | 1,0,2,5,4,6,11,9,8,10,13,12,14 | 0 | – | – | – | – | – |
| 17 | 23 | 1 | 0 | [] | 1/1 | 6 | – |
| 25 | 27 | 1 | 0 | [] | 0/0 | – | – |
| 29 | 29 | 1 | 0 | [] | 0/0 | – | – |
| 31 | 30 | 1 | 0 | [] | 0/0 | – | – |
| 32 | 47 | 1 | 1 | [7] | 1/1 | 7 | 7: 1/1 |
| 33 | 39 | 1 | 1 | [7] | 0/0 | – | – |
| 34 | 35 | 1 | 1 | [7] | 0/0 | – | – |
| 35 | 33 | 1 | 1 | [7] | 0/0 | – | – |
| 36 | 32 | 1 | 0 | [] | 0/0 | – | – |
| 37–47 | 其余 TID 5 + 43/41/40/42/45/44/46 | 0 | – | – | – | – | – |

可以读出：

1. **POC 31（IDR）的 AU 里写出 luma 新滤波器（id 7）**：`sh_num_alf_aps_ids_luma=1`、`sh_alf_aps_id_luma=7`，该 AU 的 APS 7 是 `alf_luma_new_filter=1, alf_chroma_new_filter=0`。
2. **chroma 的 APS 由下一帧造出**：POC 15 的 AU 写出 APS 6（`alf_luma_new_filter=0, alf_chroma_new_filter=1`），而它自己的 `numAps=0` —— 不复用任何 temporal luma 滤波器，只用固定集。POC 15 的 slice header 已经以 `sh_alf_aps_id_chroma=6` 引用它，紧随的 POC 7 同样引用 6。也就是说 **chroma APS 在写出的同一帧内立即被使用；而 luma 的 temporal APS 7 在 POC 15/7/3 上都没被引用（`numAps=0`）** —— luma 与 chroma 的复用决策完全独立。
3. **POC 32 的 `en=1, nAPS=0`**：整帧只用固定滤波器集，其 AU 也不带任何 APS。POC 27/29/30 同理。
4. **POC 47 是新 luma+chroma 滤波器的作者**（AU 内带 `APS 7: 1/1`），其后 POC 39/35/33 连续三帧以 `nAPS=1, luma=[7]` 复用它 —— 这正是参数集复用生效的窗口。
5. **`ALFSpeed` 的阈值**（`hdr_fast.txt` / `hdr_sp2.txt`）：`ALFSpeed=1` 时 `maxTLayers - 1 = 5`，故 **TID 5 的帧必然 `sh_alf_enabled_flag = 0`**（实测 12 个 TID-5 帧全为 0），而 **TID 4 不受跳过规则约束**，由 slice 级 RD 决定 —— 实测 10 个 TID-4 帧里只有 POC 29 与 POC 33 打开。这说明 `isSkipAlfForFrame` 与 slice 级 `costOff <= costMin` 是两个独立的闸门：前者是"能否处理"，后者是"处理完是否值得进入码流"。`ALFSpeed=2` 时阈值下移到 TID ≥ 4（`hdr_sp2.txt`：TID 0/1/2 全开，TID ≥ 4 全关，TID 3 视 RD 而定 —— 8 个 TID-3 帧里 POC 27/35 打开）。

### 6.3 trace 的 POC 标记与 AU 分组不是一回事

实测中观察到 `D_HEADER` 里偶发出现"孤立的 POC 标记"（如 `hdr48.txt` 中 POC 47 的标记出现在 POC 7 之前，而它自己的 AU 在文件末尾）。核实后确认这**不是** APS 归属错位，而是 trace 语义问题：

- `=========== POC: N`（`EncSlice.cpp:1543`）由 `encodeSliceData` 在 `finalizePicture` 里发出，而 `xWriteParameterSets` / `codeSliceHeader` 在 `xWritePicture` 里发出。前者是**编码完成顺序**，后者是**AU 输出顺序**。
- 二者在本例中确实不同：`Threads=1` 时 `fetchOnEveryPass` 为真（`EncGOP.cpp:438-439`），于是 `findNextPicToStart` 会按 temporal layer 排序取帧（`:381-420`），TLayer 1 的 POC 47 在 POC 15 重建完成后就抢先开始，早于 TLayer 2 的 POC 7。所以 `POC: 47` 的标记出现在记录 POC 7 之前，而 POC 47 自己的 AU（APS 在 `hdr48.txt:2037`、slice header 在 `:2088`）输出得晚得多。
- 反过来，**AU 内 APS 先于 slice 这一点在所有实测文件里都成立**，例如 POC 47 的 AU：APS id 7（`:2037`）→ 该 AU 的 slice header `ph_pic_order_cnt_lsb=47`（`:2096`）→ `sh_alf_aps_id_luma=7`（`:2109`）。

两点结论：(1) `D_HEADER` trace 不能用来划分 AU，需要额外的 AU 边界标记；(2) **开始编码顺序（start order）≠ 编码号顺序（codingNum order）**，但 APS 链由 `xSetupPicAps` 按 codingNum 建表（`EncGOP.cpp:1627` 的 `xInitPicsInCodingOrder`），因而与 start order 无关 —— 这正是 6.7 中 `Threads=1/2/4` 输出逐字节相同的原因。

### 6.4 开关的 RD 影响（`testsrc2` 64 帧，QP 27 / 37，`--Threads=0 --qpa 0`）

| 配置 | QP27 bitrate | QP27 YUV-PSNR | QP37 bitrate | QP37 YUV-PSNR |
|---|---:|---:|---:|---:|
| ALF+CCALF+TempPred | 178.88 | 41.5956 | 46.29 | 34.5715 |
| `--ALF=0` | 178.44 | 41.5857 | 45.96 | 34.5604 |
| `--CCALF=0` | 178.87 | 41.5956 | 46.25 | 34.5715 |
| `--ALFTempPred=0` | 178.83 | 41.5931 | 46.29 | 34.5715 |
| `--ALFSpeed=2` | 178.58 | 41.5857 | 46.27 | 34.5700 |

（bitrate 单位 kbps，序列极短，仅用于看方向性。）

观察：

- `--ALFSpeed=2` 与 `--ALF=0` 的 PSNR 基本持平（QP27：41.5857 vs 41.5857；QP37：34.5700 vs 34.5604），bitrate 只差 0.07%。原因可直接从跳过规则读出：本配置 `maxTLayers = 6`，`--ALFSpeed=2` 使 `TLayer >= 4` 的帧整帧跳过（`isSkipAlfForFrame` 的 `6 - 2 <= TLayer`），而在 `gop32.cfg` 的 32 帧 GOP 里 TID ≥ 4 的帧占绝大多数。实测（`hdr_sp2.txt`）：TID 0/1/2 全开，TID ≥ 4 全关，TID 3 视 RD 而定（POC 27/35 开，POC 3/11/19/43 关）。
- `--ALFTempPred=0` 在 QP27 下 bitrate 略高（+0.05%）、PSNR 略低，说明**复用带来的收益在这个短序列上很小**，主要因为 APS 重发在 352x240 短序列里占比不高。在长序列/高分辨率上这一项才会显现。
- `--CCALF=0` 与默认几乎无差异：本素材的 chroma 与 luma 相关性不足以让 CC-ALF RD 通过。这与 6.5 的结论一致。

### 6.5 CC-ALF 的触发条件（`ccsrc` 40 帧，QP 37 / 45）

`hdr_cc2.txt`（QP 37，`randomaccess_slow`，`Threads=1`）：**只有 POC 31（IDR）一帧**打开了 CC-ALF：

```
POC 31: sh_alf_enabled=1, sh_alf_cb_enabled=1, sh_alf_cr_enabled=1,
        sh_cc_alf_cb_enabled=1 @APS 7, sh_cc_alf_cr_enabled=1 @APS 7
        该 AU 的 APS 7: alf_luma_new_filter=1, alf_chroma_new_filter=1,
                        alf_cc_cb_filter_signal_flag=1, alf_cc_cr_filter_signal_flag=1
POC 23: sh_alf_enabled=1, nAPS=1, luma=7,
        sh_alf_cb_enabled=0, sh_alf_cr_enabled=0,
        sh_cc_alf_cb_enabled=0, sh_cc_alf_cr_enabled=0
其余 38 帧: sh_alf_enabled=0
```

即 40 帧里只有 2 帧启用 ALF（POC 31、23），而 CC-ALF 只在 POC 31 启用。QP 45 下同样如此（`hdr_cc45.txt`：4 帧启用 ALF，CC-ALF 仍只出现在 POC 31）。**CC-ALF 只在 I 帧打开了**。这符合 `deriveCcAlfFilter` 的结构 —— 它对整帧训练（`maxTrainingIterCount = 15`，`:6133`），并在 `bestUnfilteredTotalCost < bestFilteredTotalCost` 时整体放弃（`:6319-6322`）。P 帧/B 帧的残差里 chroma 与 luma 的相关性已经很高，CC-ALF 的额外 bits 不划算。这一点与 `changelog.txt:236` 提到的 `m_ccalfQpThreshold` 参数被移除相呼应：现在没有按 QP 的阈值，纯 RD 决定。

**这也解释了为什么 `sh_cc_alf_*` 全部挂在 IDR 的 luma APS id 7 上**：`xSetupCcAlfAPS` 在 `ccAlfCbApsId < 0` 时取 `m_apsIdStart`（`:1660-1661`），而 IDR 的 `m_apsIdStart` 已被 luma 新滤波器设成 7（`:5348`）。所以 CC-ALF 天然地"寄生"在 luma 的新 APS 上，不会单独占用一个 id。同一份 APS 的四个 new-filter 标志都为 1，正是 5.3 所述"共 id"设计的直接体现。

### 6.6 IFP 模式的差异（`ccsrc`，`--IFP=1 --MaxParallelFrames=1`）

`hdr_ifp.txt`（`ccsrc` 40 帧，`--Threads=4 --MaxParallelFrames=1 --IFP=1`）：

- **每个 slice 的 `sh_alf_enabled_flag` 都是 1**（40/40 帧）。验证了 5.1 的结论：IFP 下 slice 级 ALF 永不关闭。
- **APS id 全部是 0**（两处 `adaptation_parameter_set_id` 都是 0，`sh_alf_aps_id_luma` 也都是 0）。原因在 `initDerivation`：`if (!m_encCfg->m_alfTempPred) newApsId = m_apsIdStart = m_encCfg->m_explicitAPSid;`（`EncAdaptiveLoopFilter.cpp:2413-2417`），而 `m_alfTempPred` 在 IFP 下被强制为 0（`vvencCfg.cpp:2217`）。于是**所有帧共用 id 0**，每帧如需要就重写。
- **`numAps` 几乎恒为 0，只有两帧为 1**：40 帧中仅 POC 31 与 POC 23 的 `sh_num_alf_aps_ids_luma = 1`（`sh_alf_aps_id_luma = 0`），其余 38 帧为 0。这两帧恰好是各自写了新 luma APS 的那两帧（`alf_luma_new_filter = 1` 出现 2 次），其余帧全部只用 16 个固定滤波器集。原因是 `getAvaiApsIdsLuma` 的开头 `if (m_encCfg->m_alfTempPred)`（`:4671`）直接拒绝了跨帧候选，所以 `numAps` 只可能来自本帧新算的那一套。

这是"参数集复用"与"并行"的直接矛盾：IFP 要求参数在行同步点前定稿，而 temporal prediction 要求参数跨帧继承，二者在 vvenc 中被配置为互斥（`vvencCfg.cpp:2217` 的 `vvenc_confirmParameter`）。

注意这个组合的代价：**40 帧里有 2 帧各自重发了整套系数（id 都是 0，互相覆盖），其余 38 帧只能退回固定滤波器集**。对照同一素材在非 IFP 下的行为（`hdr_cc2.txt`，`randomaccess_slow` + `Threads=1`）：POC 23 以 `nAPS=1, luma=7` 复用了 POC 31 写出的 APS 7，**不需要重发系数**。也就是说同一素材上，IFP 让"复用"退化为"要么重发、要么退回固定集"。

### 6.7 位精确性（bit-exactness）

- `randomaccess_medium` + `gop32`，80x44/15 帧，`--Threads=0/1/4`：`Threads=1` 与 `Threads=4` 的输出**逐字节相同**（`md5 980594ba…`），`Threads=0` 不同（`md5 001d0b63…`）。
- `randomaccess_slow` + `--QP=26`（`qpa 0`），352x240/48 帧，`Threads=1/2/4` 三者输出**逐字节相同**（`md5 61f377cd…`）。
- `--ALF=0` 时 `Threads=0` 与 `Threads=1` 仍然不同，说明**这一差异不是 ALF 引起的**，而是 `Threads=0` 与 `Threads>0` 在 `WppBitEqual`/`CabacInitPresent`/`SAOEncodingRate` 等工具上的默认值差异（`vvencCfg.cpp:968-973`）。

对 ALF 而言更重要的是：**`Threads>0` 的三种帧并行度给出相同码流**，说明 APS 链在 MT 下的 `xSyncAlfAps` / `xSetupPicAps` 选择是确定性的（依赖 coding order 与 `tid`，不依赖运行时线程调度）。

---

## 7. 当前行为总结与改进建议

### 7.1 已确立的行为（源码 + 实测一致）

| 问题 | 结论 | 依据 |
|---|---|---|
| 参数何时产生 | 帧内 CTU 流的 `ALF_DERIVE_FILTER` 阶段；常规模式在最后一个 CTU 后 | `EncSlice.cpp:1254-1305` |
| 参数如何进码流 | 先写 APS（按 changed flag，id 升序），再写 slice header；APS 在 AU 内位于 slice 之前 | `EncGOP.cpp:2546-2548, 2584-2601` |
| 如何进参考 | `reconstructCTU` 直接写 `PIC_RECONSTRUCTION` buffer，inter prediction 读同一 buffer | `EncAdaptiveLoopFilter.cpp:2144`、`InterPrediction.cpp:826` |
| 何时可被引用 | `finalizeReference` 之后（`isReconstructed`），APS 同时转入全局列表 | `EncPicture.cpp:114-158`、`EncGOP.cpp:735` |
| 复用的候选集 | 上一帧 `picApsMap` 快照，按 id 环遍历，过滤 `temporalId <= TLayer` | `EncAdaptiveLoopFilter.cpp:4669-4697` |
| 复用链断点 | IRAP（MT）、`ALFTempPred=0`、IFP | `EncGOP.cpp:1512-1516`、`EncAdaptiveLoopFilter.cpp:2382` |
| 分量差异 | luma 多 APS + 25 类；chroma 单 APS + alternatives；CC-ALF 寄生在 luma/chroma APS id 上 | 2.2、5.3 |
| IFP 下 | slice 级 ALF 恒开；APS id 恒为 `explicitAPSid`（本例 0）；`numAps` 仅在本帧新算滤波器时非 0 | 5.1、6.6 |

### 7.2 建议

**A. 让 `xWriteParameterSets` 的外层条件与 APS 的独立性对齐。**
`xWriteParameterSets` 的外层条件是 `alfEnabled[Y] || ccAlfCbEnabled || ccAlfCrEnabled`（`EncGOP.cpp:2584`）。当前之所以安全，是因为所有 `setChangedFlag` 都只出现在"ALF 保持开启"的分支里。但这条因果链是布局级的，不是声明级的。建议把外层条件改为"本 picApsMap 中存在 changed 的 ALF APS"，即：
```cpp
for ( int apsId = 0; apsId < ALF_CTB_MAX_NUM_APS; apsId++ ) { ... }   // 去掉外层 if
```
APS 是独立 NAL，是否写出本就不该取决于本 AU 的 slice 是否启用 ALF。这样既去掉了一个隐式耦合，也让"删掉 changed flag 的时机"（当前在写出后）成为唯一决定点。

**B. 给 APS 输出加 AU 边界与 POC 标记。**
`D_HEADER` 的 `=== POC: N` 只在 `encodeSliceData` 处发出（`EncSlice.cpp:1543`），而 APS 与 slice header 在 `xWritePicture` 里发出，且两条路径的 POC 顺序可能不同（6.3）。建议在 `EncGOP::xWriteAPS` 加 `DTRACE(g_trace_ctx, D_HEADER, "=== APS id=%d (POC %d) ===\n", aps->apsId, aps->poc)`，并在 `xWritePicture` 加一条 AU 起始标记。否则任何基于 trace 的 APS 时间线分析都会错位。

**C. 消除 `chromaApsId = -1` 的隐式不变量。**
`CABACWriter.cpp:181` 直接下标 `cs.slice->alfAps[cs.slice->chromaApsId]`。安全完全依赖 `alfEnabled[Cb]||alfEnabled[Cr]` 与 `chromaApsId >= 0` 同生同灭。建议改为：
```cpp
if (isChroma(...) && cs.slice->chromaApsId >= 0) { ... }
```
或把三者的赋值收敛到一个 helper（当前 `:5518-5519` 设、`:5532-5535` 清，分散在两个相邻分支）。

**D. 显式化"luma ALF 关闭则 CC-ALF 必关"的约束。**
`deriveCcAlfFilter` 的早退（`:6119-6123`）与 `alfEncoderCtb` 的 `memset(cs.slice->alfEnabled, 0, ...)`（`:5318-5324`）各自保证这一点，但分散在两处、两个阶段。建议抽成 `isCcAlfAllowed(slice)` 单点判断，并在 `xSetupCcAlfAPS` 入口 CHECK。

**E. 复用候选集加"适用性"判据。**
`getAvaiApsIdsLuma` 只按 `temporalId`/`layerId` 过滤（`:4681`），不参考距离、不参考 POC。`alfEncoderCtb` 随后对所有候选 × {用/不用新滤波器} 做穷举（`:5090-5100`，复杂度 O(apsIds.size())），同时**这些滤波器可能非常旧**。建议对 `curAPS->poc` 与当前 `poc` 的距离设上限（例如超过一个 GOP 就不再作为候选）。`PicApsGlobal` 已保存 `poc`（`Picture.h:128-135`），信息现成。

**F. 统一三处"APS 链断点"定义。**
一处用 `mtPicParallel && pic->slices[0]->isIntra()`（`EncGOP.cpp:1512`），一处用 `mtPicParallel && slice.isIntra()`（`:616`），一处用 `!layerIdx && (pendingRasInit || isIDRorBLA() || (CRA && m_craAPSreset))`（`EncAdaptiveLoopFilter.cpp:2382`）。三者语义不同，而且**前两处只看线程模式、第三处只看 NAL 类型** —— 单线程下 IDR 不清 `picApsMap` 的传播（但 `initDerivation` 会清），MT 下 IRAP 不设 reference（`initDerivation` 也会清），行为需要逐组合推演才能确认。建议合并为 `isApsChainReset(slice, cfg)` 并在三处共用。

**G. 为 ALF 参数链补集成测试。**
`cmake/modules/vvencTests.cmake` 里的 ALF 覆盖只有 `randomaccess_tooltest` 的整链路 bit-exact 比较（`compare_output-tooltest`），以及 `test/vvenc_unit_test` 的 kernel 级比较（`check_filterBlkCcAlf`、`check_getPreBlkStatsAccum`，`vvenc_unit_test.cpp:6192/6552`）。**没有测试验证**：
- APS 链在 IRAP 处切断；
- `numAps = 0` 的帧只用固定滤波器集；
- IFP 下 slice 级 ALF 恒开；
- `ALFSpeed` 的跳过阈值与 TID 的关系。

建议加一个只检查语法元素序列（不比对像素）的测试：用 `--tracerule=D_HEADER` 跑固定素材，grep `sh_alf_enabled_flag` / `sh_num_alf_aps_ids_luma` / `adaptation_parameter_set_id` 的期望序列。仓库已有 `test/data/RTn23_80x44p15_f15.yuv` 与 `test/data/RTn23.cfg`，成本很低。当前这种测试若存在，本文 6.2–6.6 的若干结论就应该是回归保护的对象。

**H. IFP 下的 slice 级 ALF 恒开值得商榷。**
`EncAdaptiveLoopFilter.cpp:5315-5316` 在 `m_ifpLines > 0` 时禁用"整帧关闭 ALF"的选项，注释说"final decision at CTU level"。实测（`hdr_ifp.txt`）中**40/40 帧**都写了 `sh_alf_enabled_flag=1`，包括在非 IFP 模式下必然关闭的高 TID 帧。这会带来固定的 slice header 开销 + 每 CTU 的 `alf_ctb_flag`。建议至少允许"当整帧所有 CTU 的 `m_ctuEnableFlag` 全为 0 时"回退到 `sh_alf_enabled_flag=0` —— 该信息在 `alfEncoderCtb` 结束时已知。（代价是需要确认 IFP 的后续行选择逻辑不依赖 slice 级 flag 已置位，见 5.1。）

**I. 清理 `m_apsIdsChroma`。**
`EncAdaptiveLoopFilter.h:403` 的 `m_apsIdsChroma` 只有 `clear()`（`:2375`）与 `push_back()`（`:4689`），没有任何读取。chroma 复用实际在 `alfEncoderCtb` 内现场扫描（`:5414`）。建议删除该成员，避免后来者误以为存在一条 chroma 候选列表。

---

## 8. 复现命令

以下命令假定在仓库根目录执行，且已按第 6 节构建出 `bin/release-static/vvencFFapp`（`-DVVENC_ENABLE_TRACING=ON`）。

```bash
# 构建（tracing 版）
cmake -S . -B build-trace -DCMAKE_BUILD_TYPE=Release -DVVENC_ENABLE_TRACING=ON
cmake --build build-trace -j"$(nproc)" --target vvencFFapp

# 6.2 / 6.3：APS 时间线（仓库自带素材）
./bin/release-static/vvencFFapp \
  -c cfg/randomaccess_medium.cfg -c cfg/gop32.cfg -c test/data/RTn23.cfg \
  -i test/data/RTn23_80x44p15_f15.yuv -f 15 --Threads=1 \
  --tracerule=D_HEADER --tracefile=hdr.txt -b out.vvc -o rec.yuv
grep -nE "POC: |Adaptation Parameter|adaptation_parameter_set_id|sh_alf" hdr.txt

# 6.4：开关对比（需先准备 testsrc2 的 cfg）
for m in ALF=1 ALF=0 CCALF=0 ALFTempPred=0 ALFSpeed=2; do
  ./bin/release-static/vvencFFapp -c cfg/randomaccess_medium.cfg -c cfg/gop32.cfg \
    -c testsrc2.cfg --Threads=0 --qpa 0 --QP=27 --$m -b "rd_$m.vvc" -o "rd_$m.yuv"
done

# 6.5：CC-ALF 触发（需 chroma-luma 强相关素材）
./bin/release-static/vvencFFapp -c cfg/randomaccess_slow.cfg -c cfg/gop32.cfg \
  -c cc2.cfg --Threads=1 --tracerule=D_HEADER --tracefile=hdr_cc.txt -b cc.vvc -o rec_cc.yuv
grep -nE "POC: |sh_alf_enabled|sh_cc_alf|alf_cc_c" hdr_cc.txt

# 6.6：IFP
./bin/release-static/vvencFFapp -c cfg/randomaccess_medium.cfg -c cfg/gop32.cfg \
  -c cc2.cfg --Threads=4 --MaxParallelFrames=1 --IFP=1 \
  --tracerule=D_HEADER --tracefile=hdr_ifp.txt -b ifp.vvc -o rec_ifp.yuv

# 6.7：位精确性
for t in 0 1 2 4; do
  ./bin/release-static/vvencFFapp -c cfg/randomaccess_slow.cfg -c cfg/gop32.cfg \
    -c s48q.cfg --Threads=$t -b "q_t$t.vvc" -o "rec_t$t.yuv"
done
md5sum q_t*.vvc
```

---

## 附录：关键常量与结构

| 名称 | 值 | 位置 |
|---|---|---|
| `ALF_CTB_MAX_NUM_APS` | 8 | `CommonDef.h:262` |
| `NUM_FIXED_FILTER_SETS` | 16 | `CommonDef.h:263` |
| `MAX_NUM_ALF_CLASSES` | 25 | `CommonDef.h:250` |
| `MAX_NUM_CC_ALF_FILTERS` | 4 | `CommonDef.h:256` |
| `VVENC_MAX_NUM_ALF_ALTERNATIVES_CHROMA` | 8 | `vvencCfg.h:83` |
| `NUM_APS_TYPE_LEN` | 3 | `CommonDef.h:279` |
| `numAps` / `lumaApsId` / `chromaApsId` / `ccAlfCbApsId` | – | `Slice.h:1215-1221` |
| `PicApsGlobal`（含 `apsMap`, `refCnt`, `initalized`） | – | `Picture.h:128-135` |
| `AlfParam`（lumaCoeff / chromaCoeff / newFilterFlag） | – | `AlfParameters.h:135-171` |
| `CcAlfFilterParam`（ccAlfCoeff / newCcAlfFilter） | – | `AlfParameters.h:173-196` |
