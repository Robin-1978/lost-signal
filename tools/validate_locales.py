#!/usr/bin/env python3
"""Validate sparse game-language overlays without changing simulation data."""

from __future__ import annotations

import json
import re
from pathlib import Path
from typing import Any, Iterator

from jsonschema import Draft202012Validator


HAN = re.compile(r"[\u3400-\u9fff]")
PLACEHOLDER = re.compile(r"\{\d+\}")


class LocaleError(RuntimeError):
	pass


def _read(path: Path) -> dict[str, Any]:
	try:
		data = json.loads(path.read_text(encoding="utf-8"))
	except (OSError, json.JSONDecodeError) as exc:
		raise LocaleError(f"{path}: cannot read JSON: {exc}") from exc
	if not isinstance(data, dict):
		raise LocaleError(f"{path}: root must be an object")
	return data


def _string_leaves(value: Any, pointer: str = "") -> Iterator[tuple[str, str]]:
	if isinstance(value, dict):
		for key, child in value.items():
			escaped = key.replace("~", "~0").replace("/", "~1")
			yield from _string_leaves(child, f"{pointer}/{escaped}")
	elif isinstance(value, list):
		for index, child in enumerate(value):
			yield from _string_leaves(child, f"{pointer}/{index}")
	elif isinstance(value, str):
		yield pointer, value


def _resolve(value: Any, pointer: str) -> Any:
	current = value
	if not pointer.startswith("/"):
		raise LocaleError(f"invalid JSON pointer: {pointer}")
	for raw in pointer[1:].split("/"):
		token = raw.replace("~1", "/").replace("~0", "~")
		try:
			current = current[int(token)] if isinstance(current, list) else current[token]
		except (KeyError, IndexError, ValueError, TypeError) as exc:
			raise LocaleError(f"JSON pointer does not exist: {pointer}") from exc
	return current


def _edge_whitespace(value: str) -> tuple[str, str]:
	return value[: len(value) - len(value.lstrip())], value[len(value.rstrip()) :]


def _content_documents(root: Path) -> Iterator[Path]:
	for path in sorted(root.rglob("*.json")):
		if "schema" in path.parts or "locales" in path.parts or path.name == "locales.json":
			continue
		yield path


def validate_locales(root: Path) -> tuple[int, int]:
	root = root.resolve()
	config_path = root / "locales.json"
	config = _read(config_path)
	config_schema = _read(root / "schema" / "locales.schema.json")
	errors = list(Draft202012Validator(config_schema).iter_errors(config))
	if errors:
		raise LocaleError(f"{config_path}: {errors[0].message}")

	supported = config["supported_locales"]
	default = config["default_locale"]
	source = config["source_locale"]
	if default not in supported or source not in supported:
		raise LocaleError("default_locale and source_locale must be supported")
	if default == source:
		raise LocaleError("default_locale must exercise a translated language overlay")

	overlay_schema = _read(root / "schema" / "locale-overlay.schema.json")
	documents = list(_content_documents(root))
	required_entries = 0
	checked_entries = 0
	for locale in supported:
		if locale == source:
			continue
		for base_path in documents:
			base = _read(base_path)
			required = {pointer: text for pointer, text in _string_leaves(base) if HAN.search(text)}
			overlay_path = root / "locales" / locale / base_path.relative_to(root)
			if not required and not overlay_path.exists():
				continue
			if not overlay_path.is_file():
				raise LocaleError(f"{overlay_path}: missing locale overlay")
			overlay = _read(overlay_path)
			errors = list(Draft202012Validator(overlay_schema).iter_errors(overlay))
			if errors:
				raise LocaleError(f"{overlay_path}: {errors[0].message}")
			translations = overlay["translations"]
			missing = sorted(set(required) - set(translations))
			if missing:
				raise LocaleError(f"{overlay_path}: missing translations {missing}")
			for pointer, translated in translations.items():
				source_text = _resolve(base, pointer)
				if not isinstance(source_text, str):
					raise LocaleError(f"{overlay_path}:{pointer}: target is not a string")
				if HAN.search(translated):
					raise LocaleError(
						f"{overlay_path}:{pointer}: translated text still contains Han characters"
					)
				if sorted(PLACEHOLDER.findall(source_text)) != sorted(PLACEHOLDER.findall(translated)):
					raise LocaleError(f"{overlay_path}:{pointer}: format placeholders changed")
				if source_text.count("\n") != translated.count("\n"):
					raise LocaleError(f"{overlay_path}:{pointer}: explicit line breaks changed")
				if _edge_whitespace(source_text) != _edge_whitespace(translated):
					raise LocaleError(f"{overlay_path}:{pointer}: leading or trailing whitespace changed")
				checked_entries += 1
			required_entries += len(required)
	return checked_entries, required_entries


def main() -> int:
	root = Path(__file__).resolve().parent.parent / "assets"
	checked, required = validate_locales(root)
	print(f"LOCALES VALID: {checked} entries checked, {required} required translations")
	return 0


if __name__ == "__main__":
	raise SystemExit(main())
