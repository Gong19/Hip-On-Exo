"""Local persistent connection settings; exclude the private JSON from handoffs."""
import json
from pathlib import Path
import secrets

PRIVATE_CONFIG = Path(__file__).resolve().parent / 'bridge_private_config.json'


def load_connection_token(path=PRIVATE_CONFIG):
    path = Path(path)
    if not path.exists():
        token = secrets.token_hex(24)
        try:
            with path.open('x', encoding='utf-8') as stream:
                json.dump({'connection_token': token}, stream, indent=2)
                stream.write('\n')
        except FileExistsError:
            pass
    try:
        config = json.loads(path.read_text(encoding='utf-8'))
        token = config['connection_token']
    except (OSError, ValueError, KeyError, TypeError):
        raise ValueError('无法读取本机固定令牌配置；请检查 bridge_private_config.json。') from None
    if not isinstance(token, str) or not token or len(token)>1024 or token != token.strip():
        raise ValueError('本机固定令牌配置无效；请检查 bridge_private_config.json。')
    return token
