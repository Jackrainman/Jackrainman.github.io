---
name: team-doc-generator
description: 实现战队文档的智能生成引擎，包含代码分析、难度分级和模板渲染功能
license: MIT
compatibility: opencode
metadata:
  audience: developers
  workflow: documentation
---

基于项目结构，完善 `./team-doc-skill/core/` 目录下的生成引擎。

## 具体实现

### 1. core/analyzer.py
实现 `CodeAnalyzer` 类：
```python
from pathlib import Path
from typing import List
from .models import CodeSnippet, KnowledgeNode

class CodeAnalyzer:
    def analyze_project(self, code_dir: Path) -> List[CodeSnippet]:
        """分析项目代码，返回代码片段列表"""
        pass
    
    def extract_knowledge(self, snippet: CodeSnippet) -> KnowledgeNode:
        """从代码片段提取知识点"""
        pass
```

- `analyze_project(code_dir: Path) -> List[CodeSnippet]`:
  - 调用 `opencode analyze --code-dir {code_dir} --format json`
  - 解析JSON，提取函数复杂度、算法模式
  - 识别关键字：dp, dfs, dijkstra, bfs 等
- `extract_knowledge(snippet: CodeSnippet) -> KnowledgeNode`:
  - 使用 OpenCode `generate` 命令提取核心算法概念

### 2. core/difficulty.py
实现 `DifficultyClassifier` 类：
```python
from typing import Literal
from .models import CodeSnippet, DifficultyLevel

class DifficultyClassifier:
    def classify(self, snippet: CodeSnippet) -> Literal["入门", "进阶", "挑战", "省选"]:
        """根据代码特征自动分类难度"""
        pass
```

- 基于代码复杂度、算法标签自动分级
- 支持读取 `config/difficulty-map.yaml` 自定义规则
- 方法：`classify(snippet: CodeSnippet) -> Literal["入门", "进阶", "挑战", "省选"]`

### 3. core/generator.py
实现 `DocGenerator` 类：
```python
from .models import KnowledgeNode, CodeSnippet, DocModule

class DocGenerator:
    def generate_module(self, knowledge: KnowledgeNode, snippet: CodeSnippet, difficulty: str) -> DocModule:
        """生成文档模块"""
        pass
```

- `generate_module(knowledge, snippet, difficulty) -> DocModule`:
  - 根据难度选择模板
  - 使用 Jinja2 渲染
  - 调用 OpenCode 生成教学叙述
- 模板变量：`{{ prereq_list }}`, `{{ code_solution }}`, `{{ complexity_time }}`, `{{ complexity_space }}`, `{{ thinking_process }}`

### 4. templates/*.md.j2
- `beginner.md.j2`: 入门模板，更多注释、逐步解释
- `advanced.md.j2`: 进阶模板，关注优化思路
- `expert.md.j2`: 挑战模板，强调数学推导

包含异常处理和重试机制。
