from pathlib import Path

from mecanum_testing.contract_audit import audit


def test_repository_contracts():
    workspace = Path(__file__).resolve().parents[3]
    result = audit(workspace)
    assert result['result_state'] == 'PASS_STATIC', result['failed_checks']
