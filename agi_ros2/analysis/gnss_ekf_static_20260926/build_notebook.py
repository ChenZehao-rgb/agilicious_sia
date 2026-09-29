#!/usr/bin/env python3
"""Build and execute the local reproducibility companion (no live changes)."""
from pathlib import Path

import nbformat
from nbclient import NotebookClient

HERE = Path(__file__).resolve().parent


def main():
    nbf = nbformat.v4
    notebook = nbf.new_notebook(cells=[
        nbf.new_markdown_cell(
            '# 2026-09-26 GNSS / EKF 静止段核查\n\n'
            '来源：`bags/hardware_20260926_161859_702449`，时长247.707秒。'
            '仅把用户确认的前100秒当作静止。默认时间窗基于MCAP记录时间相对metadata起点；'
            '传感器比较与积分使用整数header采样时间。无外部位置真值，漂移不等于绝对定位误差。\n\n'
            '原始MCAP已重新校验CRC、全话题计数，并逐字段比较全部11张缓存表。'
            '本notebook默认复用该验证记录，随后重算分析和图。'
            '要再次做完整原包验证，将下面的`RUN_RAW_VERIFICATION`设为True。\n\n'
            '依赖：numpy、pandas、matplotlib、scipy、pyyaml；原包验证另需mcap、mcap-ros2-support。'
            '执行notebook另需nbformat、nbclient和ipykernel。'),
        nbf.new_code_cell(
            'from pathlib import Path\nimport contextlib, io, json, runpy\n'
            'import pandas as pd\nfrom IPython.display import Image, display\n'
            'HERE = Path.cwd()\n'
            'if not (HERE / "jump_analysis.py").exists():\n'
            '    HERE = HERE / "agi_ros2/analysis/gnss_ekf_static_20260926"\n'
            'assert (HERE / "jump_analysis.py").exists()\n'
            'RUN_RAW_VERIFICATION = False\n'
            'if RUN_RAW_VERIFICATION:\n'
            '    runpy.run_path(str(HERE / "verify_source.py"), run_name="__main__")\n'
            'validation = json.loads((HERE / "source_validation.json").read_text())\n'
            'assert validation["stored_crcs_checked"] and validation["all_counts_match_metadata"]\n'
            'print("Original bag verified:", validation["total_messages"], "messages")\n'
            'display(pd.DataFrame(validation["cached_tables_verified"]).T)'),
        nbf.new_code_cell(
            'for filename in ["gnss_quality.py", "imu_audit.py", "ekf_audit.py", "jump_analysis.py"]:\n'
            '    with contextlib.redirect_stdout(io.StringIO()):\n'
            '        runpy.run_path(str(HERE / filename), run_name="__main__")\n'
            '    print("Recomputed", filename)\n'
            'jump = json.loads((HERE / "jump_results.json").read_text())\n'
            'gnss = json.loads((HERE / "gnss_quality.json").read_text())\n'
            'imu = json.loads((HERE / "imu_results.json").read_text())\n'
            'ekf = json.loads((HERE / "ekf_audit_metrics.json").read_text())'),
        nbf.new_markdown_cell(
            '## 相同静止时窗的位置比较\n\n'
            'EKF约5.798秒才初始化，比较用[6,100)秒两话题各自的全部有效消息。'
            '各话题的首末采样时刻相差不超过一个导航周期；不是逐样本误差比较。'
            '图用采样时间，统计窗口用记录时间。'),
        nbf.new_code_cell(
            'display(pd.DataFrame(jump["common_static_window_6_100"]).T)\n'
            'display(Image(filename=str(HERE / "static_positions.png")))'),
        nbf.new_markdown_cell(
            '## 每次导航更新时的跳变\n\n'
            '用rtk_stamp推进识别接受的导航更新，排除初始化、跨reset和异常dt。'
            '近似校正量 = 本帧位置 − 上帧位置 − 上帧速度×dt − 0.5×上帧加速度×dt²。'
            '它是当前输出时刻的校正代理量，包含迟到观测回溯校正后的再传播效应，'
            '不是内部精确的Kalman增量或innovation。非更新帧的同一计算用于检验近似误差。'),
        nbf.new_code_cell(
            'display(pd.DataFrame(jump["windows"]["0_100"]["update"]).T)\n'
            'display(pd.DataFrame(jump["windows"]["0_100"]["no_update"]).T)\n'
            'display(Image(filename=str(HERE / "corrections_and_failure.png")))'),
        nbf.new_markdown_cell(
            '## 完整核查与限制\n\n'
            '- `README.md`：操作优先级和关键结论。\n'
            '- `gnss_notes.md` / `gnss_quality.json`：GPS漂移、位置/速度一致性、坐标重建。\n'
            '- `imu_findings.md` / `imu_results.json`：静止IMU、时序和动态航向核查。\n'
            '- `ekf_audit_notes.md` / `ekf_audit_metrics.json`：后段故障链、当前代码噪声离散化风险。\n\n'
            '本包没有完整fusion运行参数或二进制身份。当前源码提供机制解释，'
            '不能据此断言录包时使用了当前hardware.yaml的全部参数。'
            '没有修改生产代码或在线参数。'),
    ], metadata={'kernelspec': {'display_name': 'Python 3', 'language': 'python', 'name': 'python3'}})
    NotebookClient(notebook, timeout=180, kernel_name='python3', resources={'metadata': {'path': str(HERE)}}).execute()
    nbformat.validate(notebook)
    nbformat.write(notebook, HERE / 'analysis.ipynb')
    assert all(cell.execution_count is not None for cell in notebook.cells if cell.cell_type == 'code')
    print('Notebook executed and validated:', HERE / 'analysis.ipynb')


if __name__ == '__main__':
    main()
