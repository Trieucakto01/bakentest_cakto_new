import sys
import re

def check_file(filename):
    with open(filename, 'r') as f:
        lines = f.readlines()

    in_function = False
    in_block = False
    block_depth = 0
    statement_seen_in_block = False

    declaration_pattern = re.compile(r'^\s*(int|uint8_t|uint16_t|uint32_t|float|char|void|bool|commandResult_t)\s+\*?[a-zA-Z_]\w*\s*(?:=.*?|\[.*?\]|[,;])')
    
    for i, line in enumerate(lines):
        line = line.strip()
        if not line or line.startswith('//') or line.startswith('#'):
            continue
            
        if '{' in line:
            block_depth += line.count('{')
            if not in_function and block_depth == 1:
                in_function = True
            
            # Start of a new block
            statement_seen_in_block = False
        
        if in_function and block_depth > 0:
            if declaration_pattern.match(line):
                if statement_seen_in_block:
                    print(f"Line {i+1}: Declaration after statement: {line}")
            elif not line.startswith('{') and not line.startswith('}'):
                statement_seen_in_block = True
                
        if '}' in line:
            block_depth -= line.count('}')
            if block_depth == 0:
                in_function = False

if __name__ == '__main__':
    check_file(sys.argv[1])
