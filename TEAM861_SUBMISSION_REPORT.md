# SICT（861）最终提交包

## 2026-11-18 更新

项目根目录的 `team861_final.zip` 已更新为通过最近六组仿真的策略。按交付优先选版，同样的 60 个布局上，交付从 587/600 到 593/600，全拆从 50/60 到 56/60，均分从 87.70 到 89.18，坠毁从 6 到 1。除 Map01 外，达到 92 分的轮次从 32 到 27。两次测试使用相同布局，出生朝向与传感器噪声沿用各自的默认采样。逐轮结果见 `TEAM861_ROBUST_SIX_GROUP_TEST_REPORT.md`。

新包使用 `final` 阶段的官方命名，包含 14 个根目录文件。包内策略与工作区源码逐字节一致，工作区和最终 ZIP 均通过官方提交检查。新包 SHA256 为 `8c7be0a61230c1c6dd819e55ed62cee360b512bdbde4357a021413c2d4994011`。上一份提交包备份在 `bench_results/team861_robust_six_groups/previous_submission/team861_final.zip`。

## 2026-11-17 提交历史

日期：2026-11-17。最终压缩包为项目根目录 `team861_final.zip`，提交阶段 `final`，队伍编号 861。`team_info.yml` 含队名 SICT、成员 lishun、liudongying、chengongbo、fengsifang；lishun 排首位，注释标明队长。官方校验字段只有 `team_number`、`team_name`、`team_members`，没有独立队长字段。

压缩包根目录含 `my_drone_eval.py`、`team_info.yml`、`req.txt`、有效策略模块及官方允许的示例模块；`image_ref.txt` 为空，使用压缩包内代码评测。未包含仿真引擎、地图、启动器、配置、日志或缓存目录。

当前采用的策略为用户指定外部目录的更新版，并加入一次针对 GPS 失效后可见炸弹的修复。依据用户固定炸弹位置、十张地图、三种出生朝向种子共 30 轮配对测试，新版处置 295/300、26/30 轮全拆、平均 85.87 分、坠毁 1 架；修复前版本为 292/300、26/30、85.29 分、坠毁 1 架。只有 Map08 种子 303 发生变化，从 6/10、49.6 分提高到 9/10、67.0 分，其余 29 轮的拆弹数、分数和坠毁数相同。Map08 仍不稳定（三轮均为 9/10），不能保证全拆。详见 `TEAM861_GPS_HISTORY_FIX_REPORT.md` 和 `bench_results/team861_gps_history_visible*/` 原始日志；更早的策略对照见 `MAP_AGNOSTIC_FIXED_COMPARISON.md`。

打包后还试验了缩小携弹路径点切换半径。Map08 种子 303 从 6/10 提高到 8/10，但种子 202 从 9/10 降到 8/10，因此按“单图全拆比例不下降”准则撤回；压缩包及当前源码保留已验证版本。实验日志保存在 `bench_results/team861_waypoint_trial*/`。

随后复现 Map08 固定位置种子 303 的 6/10 失败轮，临时传感器日志显示终局有 3 架仍携弹、1 枚已发现未抓取，零坠毁；主要是长距离返航与任务拥堵。诊断日志在 `bench_results/team861_map08_seed303_diagnostic/`，临时打印已删除。又试了仅在 LiDAR 显示开阔、携弹且非抓取/交付时提高约 12% 目标巡航速度：Map06 种子 303 从 8/10 到 9/10，Map08 种子 303 从 6/10 到 7/10，但 Map08 种子 101 从 9/10 降到 8/10，因此也撤回。日志在 `bench_results/team861_open_loaded_trial/` 和 `team861_open_loaded_more/`。这些结果提示不能只根据一张图的一轮提升修改全局运动参数。

最终工作区与 `team861_final.zip` 均运行官方 `scripts/check_submission.py` 检查；比赛代码仅位于 `src/swarm_rescue/solutions/`。此文件不随提交包上传。
