"""Check Python syntax and inline local Markdown link targets without imports.

No hardware access, external URL checks, Markdown rendering, or firmware builds.
"""

import ast
from pathlib import Path
import re
import sys
from urllib.parse import unquote, urlsplit


ROOT = Path(__file__).resolve().parents[1]
IGNORED = {".git", ".venv", "__pycache__"}
INLINE_LINK = re.compile(r"!?\[[^\]\n]*\]\((<[^>]+>|[^)\s]+)\)")


def source_files(suffix):
    return sorted(path for path in ROOT.rglob(f"*{suffix}")
                  if not IGNORED.intersection(path.relative_to(ROOT).parts))


def main():
    errors = []
    python_files = source_files(".py")
    markdown_files = source_files(".md")
    link_count = 0
    for path in python_files:
        try:
            ast.parse(path.read_text(encoding="utf-8-sig"), filename=str(path))
        except (SyntaxError, UnicodeError) as error:
            errors.append(f"{path.relative_to(ROOT)}: {error}")
    for path in markdown_files:
        text = path.read_text(encoding="utf-8-sig")
        # Ignore illustrative Markdown inside fenced code blocks.
        text = re.sub(r"```.*?```", "", text, flags=re.DOTALL)
        for match in INLINE_LINK.finditer(text):
            target = match.group(1).strip("<>")
            parts = urlsplit(target)
            if parts.scheme or parts.netloc or not parts.path:
                continue
            link_count += 1
            resolved = (path.parent / unquote(parts.path)).resolve()
            if not resolved.is_relative_to(ROOT):
                errors.append(f"{path.relative_to(ROOT)}: link outside repo: {target}")
            elif not resolved.exists():
                errors.append(f"{path.relative_to(ROOT)}: missing target: {target}")
    for error in errors:
        print(error, file=sys.stderr)
    print(f"Parsed {len(python_files)} Python files; checked {link_count} local "
          f"links in {len(markdown_files)} Markdown files; {len(errors)} errors.")
    return 1 if errors else 0


if __name__ == "__main__":
    sys.exit(main())
