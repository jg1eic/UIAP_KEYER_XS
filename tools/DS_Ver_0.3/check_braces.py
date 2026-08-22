import sys
p='src/main.cpp'
bal=0
with open(p,'r',encoding='utf-8') as f:
    for i,line in enumerate(f,1):
        bal += line.count('{') - line.count('}')
        if bal<0:
            print(f'NEGATIVE at line {i}: bal={bal}')
            sys.exit(0)
print(f'FINAL BAL={bal}')
