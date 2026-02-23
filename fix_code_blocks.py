import re
from pathlib import Path

LANGUAGE_PATTERNS = {
    'c': [r'#include', r'int\s+main', r'void\s+\w+\(', r'printf', r'typedef', r'uint\d+_t', r'struct\s+'],
    'python': [r'^def\s+', r'^class\s+', r'^import\s+', r'^from\s+'],
    'cpp': [r'std::', r'cout', r'#include\s*<iostream>'],
}

def detect_language(code):
    code_lines = code.strip().split('\n')[:8]
    sample = '\n'.join(code_lines)
    scores = {}
    for lang, patterns in LANGUAGE_PATTERNS.items():
        score = sum(1 for p in patterns if re.search(p, sample, re.MULTILINE))
        if score:
            scores[lang] = score
    return max(scores, key=scores.get) if scores else 'c'

def process_file(path):
    with open(path, 'r', encoding='utf-8') as f:
        content = f.read()
    
    new_content = content
    offset = 0
    
    for match in re.finditer(r'```\s*\n', content):
        start = match.start()
        end = match.end()
        
        code_start = end
        code_end = content.find('```', code_start)
        
        if code_end == -1:
            continue
            
        code = content[code_start:code_end]
        
        if code.strip():
            lang = detect_language(code)
            old = f'```\n{code}```'
            new = f'```{lang}\n{code}```'
            new_content = new_content.replace(old, new, 1)
    
    if new_content != content:
        with open(path, 'w', encoding='utf-8') as f:
            f.write(new_content)
        return True
    return False

def main():
    docs_dir = Path('docs')
    md_files = list(docs_dir.rglob('*.md'))
    
    modified = []
    for f in md_files:
        if process_file(f):
            modified.append(str(f))
    
    print(f'处理 {len(md_files)} 文件，修改 {len(modified)} 个')

if __name__ == '__main__':
    main()
