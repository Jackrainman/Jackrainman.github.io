---
name: team-doc-linter
description: 实现战队文档的规范检查与自动修复功能，确保输出符合战队标准
license: MIT
compatibility: opencode
metadata:
  audience: developers
  workflow: documentation
---

实现 `./team-doc-skill/core/linter.py` 和 `./team-doc-skill/scripts/fix-doc.py`。

## core/linter.py
实现 `TeamDocLinter`：

### 结构检查
- `check_sections(content) -> bool`: 必需6章节（前置知识、题目描述、解题思路、参考实现、复杂度分析、拓展思考）
- `check_heading_levels(content) -> List[str]`: 一级`##`，二级`###`，禁止`#`一级标题

### 内容检查
- `check_code_blocks(content) -> bool`: ```cpp + `#include <bits/stdc++.h>` 或标准头文件
- `check_prereq_links(content) -> bool`: 前置知识包含内部链接
- `check_complexity_format(content) -> bool`: LaTeX格式 `$O(n^2)$`

### 元数据检查
- `check_front_matter(content) -> Dict`: difficulty, tags, author, date

## core/validator.py
实现 `PipelineValidator`：
- `validate(doc_path) -> ValidationReport`
- 返回 `is_valid`, `errors`, `warnings`, `suggestions`

## scripts/fix-doc.py
命令行工具：
```bash
python scripts/fix-doc.py --input ./tmp/doc.md --output ./fixed/doc.md [--auto-fix]
```

## 集成
修改 `generator.py`，在 `generate()` 后自动调用 `linter.validate()`。

提供 pytest 单元测试。
