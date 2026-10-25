"""Private knowledge exports must not leak credentials or change source files."""
import importlib.util
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location(
    'knowledge_package', ROOT / 'tools/knowledge_base/package_drcda_knowledge_base.py')
PACKAGE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(PACKAGE)


def test_redacts_url_token_in_every_occurrence():
    token = 'abcdef0123456789abcdef0123456789'
    output = PACKAGE.redact(f'http://example.invalid/?token={token}\n{token}')
    assert token not in output
    assert '[REDACTED]' in output


def test_redacts_numeric_password_without_a_hardcoded_value():
    assert '987654' not in PACKAGE.redact('密码是987654')
    assert '876543' not in PACKAGE.redact('password: 876543')


def test_transcript_keeps_roles_and_text():
    output = PACKAGE.Builder.transcript([
        dict(timestamp='2026-10-25T00:00:00Z', role='user', text='example', nontext_blocks=0)
    ], 'test')
    assert 'user' in output and 'example' in output


def test_builder_defaults_resolve_repository_not_task_directory():
    assert PACKAGE.REPO == ROOT
    assert (PACKAGE.HERE / 'knowledge_base_template/00_START_HERE.md').is_file()
