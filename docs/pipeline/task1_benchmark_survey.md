# 任务1：Benchmark 可借鉴内容调研

## 目标
调研 CARLA Leaderboard、ScenarioRunner、Bench2Drive、LMDrive/LangAuto 中可借鉴的路线、场景和指标。

## 总体结论
本项目场景与评测体系采用：

```text
LMDrive/LangAuto 接入方式
+ CARLA Leaderboard 路线与基础安全指标
+ ScenarioRunner 触发条件与 actor 编排思想
+ Bench2Drive 短路线、单能力、多场景评测思想
```

## CARLA Leaderboard
主要借鉴 route XML、route completion、driving score、infraction penalty、collision、red light、stop sign、route deviation、timeout、blocked 等指标。

可借鉴场景：route following、intersection turn、traffic light crossing、obstacle in lane、static cut-in、pedestrian emerging、leading vehicle sudden brake、vehicle invading lane。

## ScenarioRunner
主要借鉴 actor 编排、触发条件和 pass/fail criteria。适合设计行人横穿、前车急刹、突发加塞、锥桶施工区、公交站等场景。

常用触发：距离触发、位置触发、时间触发、速度触发、TTC 触发。

## Bench2Drive
主要借鉴短路线、单能力场景、多场景组合、多能力统计的思想。第一阶段不直接切换 pipeline，只参考其 cut-in、overtaking、detour、emergency braking 等场景设计。

## LMDrive/LangAuto
作为主线框架，负责 language-guided closed-loop driving。东风中文语音指令应转为两类输出：

```text
标准化自然语言 instruction：输入 LMDrive；
结构化 JSON intent：输入任务状态机、指标评测和安全监督模块。
```
