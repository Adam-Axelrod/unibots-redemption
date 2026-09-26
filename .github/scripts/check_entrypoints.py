#!/usr/bin/env python3
"""Check that every node is registered in its package's setup.py.

Forgetting the `console_scripts` line is the mistake everyone in this repo
makes exactly once: the file builds fine, and then `ros2 run` says the
executable does not exist. This catches it in CI instead.

It checks both directions:

    1. every module with a main() is registered   (you wrote a node, forgot setup.py)
    2. every registered entry point actually exists (you typo'd the module name)

Pure static analysis -- it parses the files rather than importing them, so it
needs no ROS and no build. Run it yourself any time:

    python3 .github/scripts/check_entrypoints.py
"""
import ast
import pathlib
import sys

SRC = pathlib.Path(__file__).resolve().parents[2] / 'ws' / 'src'

# Modules that live in a package but are libraries, not nodes. A library has no
# main(), so it is skipped automatically -- this list is only for the odd case
# where a helper does define main() and genuinely should not be an executable.
NOT_NODES = set()


def console_scripts(setup_py):
    """Pull the console_scripts list out of a setup.py without executing it."""
    tree = ast.parse(setup_py.read_text())
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call):
            continue
        for kw in node.keywords:
            if kw.arg != 'entry_points':
                continue
            for k, v in zip(kw.value.keys, kw.value.values):
                if getattr(k, 'value', None) == 'console_scripts':
                    return [e.value for e in v.elts]
    return []


def defines_main(py_file):
    tree = ast.parse(py_file.read_text())
    return any(isinstance(n, ast.FunctionDef) and n.name == 'main'
               for n in tree.body)


def main():
    problems = []

    for setup_py in sorted(SRC.glob('*/setup.py')):
        pkg_dir = setup_py.parent
        pkg = pkg_dir.name
        module_dir = pkg_dir / pkg
        if not module_dir.is_dir():
            continue

        entries = console_scripts(setup_py)

        # "name = module.path:function" -> module.path
        registered = {}
        for entry in entries:
            name, _, target = (p.strip() for p in entry.partition('='))
            module, _, func = target.partition(':')
            registered[module] = (name, func, entry)

        # --- direction 1: a node that nobody registered ----------------------
        for py_file in sorted(module_dir.glob('*.py')):
            if py_file.name == '__init__.py':
                continue
            module = f'{pkg}.{py_file.stem}'
            if module in NOT_NODES or not defines_main(py_file):
                continue
            if module not in registered:
                rel = py_file.relative_to(SRC.parent.parent)
                problems.append(
                    f'{rel} defines main() but is not in {pkg}/setup.py.\n'
                    f'    `ros2 run {pkg} {py_file.stem}` will fail with '
                    f'"No executable found".\n'
                    f'    Add this under console_scripts:\n'
                    f"        '{py_file.stem} = {module}:main',")

        # --- direction 2: a registration pointing at nothing -----------------
        for module, (name, func, entry) in registered.items():
            _, _, stem = module.partition('.')
            py_file = module_dir / f'{stem}.py'
            rel_setup = setup_py.relative_to(SRC.parent.parent)
            if not py_file.exists():
                problems.append(
                    f"{rel_setup} registers '{entry}' but "
                    f'{pkg}/{stem}.py does not exist.')
            elif not any(isinstance(n, ast.FunctionDef) and n.name == func
                         for n in ast.parse(py_file.read_text()).body):
                problems.append(
                    f"{rel_setup} registers '{entry}' but "
                    f'{pkg}/{stem}.py has no {func}() function.')

    if problems:
        print('Node registration problems:\n', file=sys.stderr)
        for p in problems:
            print(f'  - {p}\n', file=sys.stderr)
        print('See the "Register new nodes in setup.py" note in CLAUDE.md.',
              file=sys.stderr)
        return 1

    print('All nodes are registered in setup.py.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
