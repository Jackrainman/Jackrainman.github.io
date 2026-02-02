---
name: team-doc-pipeline
description: 提供命令行接口和完整的自动化流水线，集成代码分析、文档生成和校验功能
license: MIT
compatibility: opencode
metadata:
  audience: developers
  workflow: documentation
---

完善 `./team-doc-skill/` 项目，提供 CLI 工具和自动化脚本。

## scripts/pipeline.py
端到端工作流：
```bash
python scripts/pipeline.py --code-dir ./src --output-dir ./docs --config ./config/team-standard.yaml --difficulty-levels 3 [--auto-fix] [--dry-run]
```

**流程步骤**：
1. **分析阶段**: 调用 `CodeAnalyzer` 扫描代码，生成 `analysis_report.json`
2. **规划阶段**: 拓扑排序知识点，确保前置知识先生成
3. **生成阶段**: 批量调用 `DocGenerator`，每个知识点生成3个难度版本
4. **校验阶段**: 并行运行 `TeamDocLinter`，生成 `validation_report.json`
5. **修复阶段**: 自动修复并重校验
6. **归档阶段**: 按命名规范重命名，生成 `index.md` 索引

## integration/opencode_hooks.py
- `OpencodeSkill` 类，可被 OpenCode 作为 skill 加载
- `generate_team_doc(code_path, options)` 供 OpenCode 内部调用
- `.opencodeskill` 命令映射配置

## 辅助脚本
- `scripts/init_topic.py`: 初始化新主题目录结构
- `scripts/batch_lint.py`: 批量检查文档合规性

## Makefile
```makefile
make setup: pip install -r requirements.txt
make generate: 运行完整流水线
make lint: 检查文档规范
```

## README.md
- 安装依赖
- 快速开始示例
- OpenCode CLI 集成示例
- 自定义模板方法
- 常见问题

## 示例数据
`examples/sample_code/` 包含3个示例cpp文件（递归、DP、图论）。
