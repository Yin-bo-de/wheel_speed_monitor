#!/usr/bin/env python3
"""KiCad 符号库小工具：解析 .kicad_sym，提取符号块，供原理图生成器嵌入。

用法：
  python3 kicad_lib.py summary <库文件> <符号名> [<符号名>...]
  python3 kicad_lib.py emit    <库文件> <符号名> [--as <库名:符号名>]

设计取向：只用标准库；S 表达式解析保留「原子 / 字符串」区别——
字符串回写时必须带引号，原子不能。不做 extends 派生解析：选型时直接避开
派生符号（summary 输出会标出 extends 来源），生成器里遇到就报错，免得嵌出
一个缺图形部件的半截符号。
"""
import copy
import sys


class Q(str):
    """带引号的字符串（序列化时回写引号）。"""
    __slots__ = ()


def tokenize(text):
    toks, i, n = [], 0, len(text)
    while i < n:
        c = text[i]
        if c.isspace():
            i += 1
        elif c in '()':
            toks.append(c)
            i += 1
        elif c == '"':
            j, buf = i + 1, []
            while j < n:
                if text[j] == '\\' and j + 1 < n:      # 保留转义序列原样
                    buf.append(text[j:j + 2])
                    j += 2
                elif text[j] == '"':
                    break
                else:
                    buf.append(text[j])
                    j += 1
            toks.append(Q(''.join(buf)))
            i = j + 1
        else:
            j = i
            while j < n and (not text[j].isspace()) and text[j] not in '()"':
                j += 1
            toks.append(text[i:j])
            i = j
    return toks


def parse(toks):
    pos = [0]

    def rec():
        assert toks[pos[0]] == '(', f'期望 ( 实际 {toks[pos[0]]!r}'
        pos[0] += 1
        node = []
        while toks[pos[0]] != ')':
            t = toks[pos[0]]
            if t == '(':
                node.append(rec())
            else:
                node.append(t)
                pos[0] += 1
        pos[0] += 1
        return node

    result = rec()
    assert pos[0] == len(toks), '文件尾部有多余记号'
    return result


def dump(node, indent=0):
    if isinstance(node, str) and not isinstance(node, Q):
        return node
    if isinstance(node, Q):
        return '"%s"' % node
    has_sub = any(isinstance(x, list) for x in node)
    if not has_sub:
        return '(' + ' '.join(dump(x) for x in node) + ')'
    lines = ['(' + dump(node[0])]
    for x in node[1:]:
        lines.append('  ' * (indent + 1) + dump(x, indent + 1))
    return '\n'.join(lines) + ')'


def load(path):
    with open(path, encoding='utf-8') as f:
        return parse(tokenize(f.read()))


def find_symbol(lib, name):
    for x in lib:
        if isinstance(x, list) and len(x) >= 2 and x[0] == 'symbol' and x[1] == name:
            return x
    raise KeyError(name)


def resolve_symbol(lib, name):
    """解析 extends 派生符号：把父符号的图形/引脚并进来，子符号的覆盖项优先。

    KiCad 库里很多器件是派生写法：子节点只写"不一样的那几项"（值、封装），
    图形和引脚都在父节点——直接用子节点会拿到一个没有引脚的空壳。
    并入父节点单位时要把单位名前缀改成子符号名：KiCad 按「符号名_单元_图形」
    命名解析单元，前缀对不上会认不出来。"""
    node = copy.deepcopy(find_symbol(lib, name))
    ext = get(node, 'extends')
    if not ext:
        return node
    parent = resolve_symbol(lib, ext[1])
    child_name = str(node[1]).split(':')[-1]
    parent_name = str(parent[1]).split(':')[-1]
    node = [x for x in node if not (isinstance(x, list) and x and x[0] == 'extends')]
    child_props = {p[1] for p in get_all(node, 'property')}
    child_keys = {x[0] for x in node if isinstance(x, list) and x}
    for p in get_all(parent, 'property'):
        if p[1] not in child_props:
            node.append(copy.deepcopy(p))
    for key in ('pin_names', 'pin_numbers', 'exclude_from_sim', 'in_bom',
                'on_board', 'embedded_fonts'):
        if key not in child_keys:
            v = get(parent, key)
            if v is not None:
                node.append(copy.deepcopy(v))
    if not get_all(node, 'symbol'):          # 子符号没带图形 -> 借父的，改名前缀
        for u in get_all(parent, 'symbol'):
            u = copy.deepcopy(u)
            u[1] = Q(child_name + str(u[1])[len(parent_name):])
            node.append(u)
    return node


def get(node, key):
    for x in node:
        if isinstance(x, list) and x and x[0] == key:
            return x
    return None


def get_all(node, key):
    return [x for x in node if isinstance(x, list) and x and x[0] == key]


def summary(lib, name):
    s = resolve_symbol(lib, name)
    props = [p[1] for p in get_all(s, 'property')]
    ext = get(s, 'extends')
    print(f'== {name}  extends={ext[1] if ext else "-"}')
    print(f'   props={list(props)}')
    for u in get_all(s, 'symbol'):
        for p in get_all(u, 'pin'):
            at = get(p, 'at')
            ln = get(p, 'length')
            nm = get(p, 'name')
            num = get(p, 'number')
            hide = 'hide' if 'hide' in p else ''
            print(f'   {u[1]}: pin {num[1]} "{nm[1]}" at ({at[1]},{at[2]},{at[3]}) '
                  f'len {ln[1] if ln else "?"} {p[1]} {hide}')


def main():
    mode, path = sys.argv[1], sys.argv[2]
    args = sys.argv[3:]
    as_name = None
    if '--as' in args:
        i = args.index('--as')
        as_name = args[i + 1]
        args = args[:i]
    lib = load(path)
    if mode == 'summary':
        for n in args:
            summary(lib, n)
    elif mode == 'emit':
        s = resolve_symbol(lib, args[0])
        if as_name:
            s[1] = Q(as_name)
        print(dump(s))
    else:
        raise SystemExit(f'未知模式 {mode}')


if __name__ == '__main__':
    main()
