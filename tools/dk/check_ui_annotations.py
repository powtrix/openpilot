#!/usr/bin/env python3
"""Check exact-commit UI annotations with real pyray; this is not a boot test.

No target module is imported or executed. Only a small annotation-expression
grammar is interpreted; calls, comprehensions and arbitrary attributes are not.
"""

import argparse
import ast
import builtins
import importlib
from pathlib import Path
import re
import subprocess
import sys
import typing


UI_ROOTS = ("openpilot/selfdrive/ui", "openpilot/system/ui")
MAX_SOURCE_BYTES = 2 * 1024 * 1024
MAX_PUSH_BYTES = 1024 * 1024
MAX_PUSH_REFS = 1024
SAFE_BUILTINS = {name: getattr(builtins, name) for name in (
  "int", "float", "str", "bool", "bytes", "object", "list", "dict", "tuple", "set", "frozenset", "type",
)}
SAFE_TYPING = {name: getattr(typing, name) for name in (
  "Any", "Optional", "Union", "List", "Dict", "Tuple", "Set", "FrozenSet", "Type", "Callable", "Iterable", "Sequence", "Mapping", "Literal", "Annotated",
)}


class CheckFailure(Exception):
  pass


def git(repo: Path, *args: str) -> bytes:
  result = subprocess.run(["git", "-C", str(repo), *args], capture_output=True, timeout=15, check=False)
  if result.returncode:
    raise CheckFailure("Cannot read the exact Git commit; annotation check was not completed.")
  return result.stdout


def pushed_commits(data: bytes, remote_name: str, remote_url: str) -> list[str]:
  if len(data) > MAX_PUSH_BYTES:
    raise CheckFailure("Pre-push input exceeds the safety limit.")
  try:
    lines = data.decode("utf-8").splitlines()
  except UnicodeError as exc:
    raise CheckFailure("Invalid pre-push input encoding.") from exc
  if len(lines) > MAX_PUSH_REFS:
    raise CheckFailure("Too many pre-push reference records.")
  ours = remote_name == "powtrix" or re.fullmatch(
    r"(?:https://github\.com/|git@github\.com:|ssh://git@github\.com/)powtrix/openpilot(?:\.git)?/?", remote_url,
    re.IGNORECASE,
  ) is not None
  commits = []
  for line in lines:
    fields = line.split()
    if len(fields) != 4 or not all(re.fullmatch(r"[0-9a-fA-F]{40}|[0-9a-fA-F]{64}", fields[i]) for i in (1, 3)):
      raise CheckFailure("Malformed pre-push reference record; refusing an unverified push.")
    _, local_sha, remote_ref, _ = fields
    if ours and remote_ref == "refs/heads/dkcarrot-wip" and set(local_sha) != {"0"}:
      if local_sha not in commits:
        commits.append(local_sha)
  return commits


def evaluate_annotation(node: ast.AST, names: dict):
  if isinstance(node, ast.Name):
    if node.id not in names:
      raise CheckFailure(f"Unsupported annotation name: {node.id}")
    return names[node.id]
  if isinstance(node, ast.Constant) and (node.value is None or node.value is Ellipsis or type(node.value) in (str, int, float, bool)):
    return node.value
  if isinstance(node, (ast.Tuple, ast.List)):
    values = [evaluate_annotation(value, names) for value in node.elts]
    return tuple(values) if isinstance(node, ast.Tuple) else values
  if isinstance(node, ast.Attribute) and isinstance(node.value, ast.Name) and not node.attr.startswith("_"):
    module = names.get(node.value.id)
    if module is typing:
      if node.attr not in SAFE_TYPING:
        raise CheckFailure(f"Unsupported typing annotation: {node.attr}")
      return SAFE_TYPING[node.attr]
    if getattr(module, "__name__", None) == "pyray":
      if node.attr not in vars(module):
        raise CheckFailure(f"Missing real pyray constructor: {node.attr}")
      return vars(module)[node.attr]
  if isinstance(node, ast.BinOp) and isinstance(node.op, ast.BitOr):
    return evaluate_annotation(node.left, names) | evaluate_annotation(node.right, names)
  if isinstance(node, ast.Subscript):
    base = evaluate_annotation(node.value, names)
    # Only trusted built-in/typing subscriptions; never a source-defined hook.
    if not any(base is value for value in (*SAFE_BUILTINS.values(), *SAFE_TYPING.values())):
      raise CheckFailure("Unsupported annotation subscription.")
    return base[evaluate_annotation(node.slice, names)]
  raise CheckFailure(f"Unsupported annotation expression: {type(node).__name__}; no source code was executed.")


def check_source(source: str, filename: str, pyray) -> list[str]:
  # ast.parse alone accepts compiler-invalid code such as duplicate arguments,
  # top-level return, and misplaced future imports. Validate import-time syntax
  # before the deferred-annotation shortcut, without executing target code or
  # writing bytecode files. Do not inherit this checker's own future flags.
  compile(source, filename, "exec", dont_inherit=True)
  tree = ast.parse(source, filename=filename)
  if any(isinstance(node, ast.ImportFrom) and node.module == "__future__" and
         any(alias.name == "annotations" for alias in node.names) for node in tree.body):
    return []
  names = dict(SAFE_BUILTINS)
  pyray_names = set()
  for node in ast.walk(tree):
    if isinstance(node, ast.Import):
      for alias in node.names:
        if alias.name in ("pyray", "typing"):
          name = alias.asname or alias.name
          names[name] = pyray if alias.name == "pyray" else typing
          if alias.name == "pyray":
            pyray_names.add(name)
    elif isinstance(node, ast.ImportFrom) and node.module in ("pyray", "typing"):
      for alias in node.names:
        if alias.name == "*":
          raise CheckFailure(f"{filename}:{node.lineno}: wildcard imports require manual annotation review.")
        name = alias.asname or alias.name
        exports = vars(pyray) if node.module == "pyray" else SAFE_TYPING
        if alias.name in exports:
          names[name] = exports[alias.name]
        if node.module == "pyray":
          pyray_names.add(name)

  failures = []

  def check(annotation):
    if annotation is None or isinstance(annotation, ast.Constant) and isinstance(annotation.value, str):
      return
    if not any(isinstance(node, ast.Name) and node.id in pyray_names for node in ast.walk(annotation)):
      return
    try:
      evaluate_annotation(annotation, names)
    except (CheckFailure, TypeError, ValueError, AttributeError) as exc:
      failures.append(f"{filename}:{annotation.lineno}: {ast.unparse(annotation)}: {exc}")

  class AnnotationVisitor(ast.NodeVisitor):
    scope = "module"

    def visit_FunctionDef(self, node):
      for arg in (*node.args.posonlyargs, *node.args.args, *node.args.kwonlyargs):
        check(arg.annotation)
      for arg in (node.args.vararg, node.args.kwarg):
        if arg is not None:
          check(arg.annotation)
      check(node.returns)
      previous, self.scope = self.scope, "function"
      for statement in node.body:
        self.visit(statement)
      self.scope = previous

    visit_AsyncFunctionDef = visit_FunctionDef

    def visit_ClassDef(self, node):
      previous, self.scope = self.scope, "class"
      for statement in node.body:
        self.visit(statement)
      self.scope = previous

    def visit_AnnAssign(self, node):
      if self.scope != "function":
        check(node.annotation)

  AnnotationVisitor().visit(tree)
  return failures


def check_commit(repo: Path, revision: str, pyray) -> tuple[str, list[str], int]:
  if not re.fullmatch(r"[0-9a-fA-F]{7,64}|HEAD", revision):
    raise CheckFailure("Expected an exact commit ID (or HEAD for a manual check).")
  commit = git(repo, "rev-parse", "--verify", f"{revision}^{{commit}}").decode().strip()
  paths = git(repo, "ls-tree", "-r", "--name-only", "-z", commit, "--", *UI_ROOTS).split(b"\0")
  if len(paths) > 4096:
    raise CheckFailure("UI file count exceeds the safety limit.")
  failures = []
  count = 0
  for encoded_path in paths:
    path = encoded_path.decode("utf-8")
    if not path.endswith(".py") or "/tests/" in path:
      continue
    object_name = f"{commit}:{path}"
    if int(git(repo, "cat-file", "-s", object_name)) > MAX_SOURCE_BYTES:
      raise CheckFailure(f"UI source exceeds safety limit: {path}")
    source = git(repo, "show", object_name).decode("utf-8")
    failures.extend(check_source(source, path, pyray))
    count += 1
  if not count:
    raise CheckFailure("No UI Python files were found in the pushed commit.")
  return commit, failures, count


def main(argv=None) -> int:
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument("--repo", type=Path, default=Path(__file__).resolve().parents[2])
  parser.add_argument("--commit")
  parser.add_argument("--pre-push", action="store_true")
  parser.add_argument("--remote-name", default="")
  parser.add_argument("--remote-url", default="")
  args = parser.parse_args(argv)
  try:
    if args.pre_push:
      revisions = pushed_commits(sys.stdin.buffer.read(MAX_PUSH_BYTES + 1), args.remote_name, args.remote_url)
    elif args.commit:
      revisions = [args.commit]
    else:
      raise CheckFailure("Provide --commit or --pre-push.")
    if not revisions:
      return 0
    try:
      pyray = importlib.import_module("pyray")
    except Exception as exc:
      raise CheckFailure(f"Real pyray is unavailable ({type(exc).__name__}); push is not verified.") from exc
    if not getattr(pyray, "__file__", None) or not callable(getattr(pyray, "Rectangle", None)):
      raise CheckFailure("A real installed pyray module is required; test doubles are not accepted.")
    for revision in revisions:
      commit, failures, count = check_commit(args.repo, revision, pyray)
      if failures:
        print("DK UI annotation check FAILED; push blocked:", file=sys.stderr)
        for failure in failures:
          print(f"  {failure}", file=sys.stderr)
        return 1
      print(f"DK UI annotation check passed: {commit[:12]} ({count} exact-commit files). Not a full boot verification.")
    return 0
  except (CheckFailure, OSError, subprocess.TimeoutExpired, UnicodeError, SyntaxError, ValueError, RecursionError) as exc:
    print(f"DK UI annotation check incomplete; push blocked: {exc}", file=sys.stderr)
    return 2


if __name__ == "__main__":
  sys.exit(main())
