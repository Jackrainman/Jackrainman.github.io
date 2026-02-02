---
name: team-doc-scaffold
description: 建立战队文档生成项目的基础脚手架，包括目录结构、配置文件和数据模型
license: MIT
compatibility: opencode
metadata:
  audience: developers
  workflow: documentation
---

你是一个专业的工程架构师。请在 `{project_dir}` 目录下创建完整的自动化战队文档生成 Skill。

## 项目结构
创建以下目录结构：
- `config/` - 配置文件目录
- `core/` - 核心模块
- `templates/` - Jinja2 模板
- `scripts/` - 脚本工具
- `requirements.txt`

## 具体文件

### 1. config/team-standard.yaml
- 强制章节检查规则
- 代码块规范（```cpp + bits/stdc++.h`）
- 前置知识标签库
- 命名规则：`{difficulty_prefix}_{topic}_{sequence:02d}.md`

### 2. config/difficulty-map.yaml
- 难度分级映射规则
- 算法标签到难度的映射（BFS/DFS=入门，DP=进阶，网络流=挑战）

### 3. core/__init__.py
- 导出所有公共模块

### 4. core/models.py (Pydantic模型)
```python
from pydantic import BaseModel
from typing import List, Optional
from enum import Enum

class DifficultyLevel(str, Enum):
    BEGINNER = "入门"
    ADVANCED = "进阶"
    EXPERT = "挑战"
    OLYMPIC = "省选"

class KnowledgeNode(BaseModel):
    name: str
    prerequisites: List[str] = []
    difficulty: DifficultyLevel
    tags: List[str] = []

class CodeSnippet(BaseModel):
    file_path: str
    complexity_time: str
    complexity_space: str
    algorithm_tags: List[str] = []
    content: str

class DocModule(BaseModel):
    title: str
    target_audience: str
    associated_code: CodeSnippet
    prerequisite_knowledge: List[str]
    content: str
```

### 5. requirements.txt
```
jinja2>=3.1.0
pydantic>=2.0.0
pyyaml>=6.0
typer>=0.9.0
pytest>=7.0.0
```

使用 Python 3.9+ 语法，提供完整的可运行代码。
