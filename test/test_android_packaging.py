"""Confere o APK real: dashboard offline completo e idêntico ao fonte público.

Execute após `bash mobile/build-apk.sh`. Sem APK, estes testes são ignorados;
um build Android não é dependência obrigatória da suíte ROS.
"""
from html.parser import HTMLParser
import os
from pathlib import Path
from zipfile import ZipFile

import pytest

ROOT = Path(__file__).resolve().parents[1]
WEB = ROOT / 'drone_inspetor/mobile_web'
APK = Path(os.environ.get('DRONE_ANDROID_APK', ROOT / 'mobile/app/build/outputs/apk/debug/app-debug.apk'))
pytestmark = pytest.mark.skipif(not APK.is_file(), reason='Gere primeiro o APK Android')


def test_apk_offline_assets_match_current_web_source():
    """Detecta APK antigo ou recurso novo esquecido no empacotamento."""
    with ZipFile(APK) as apk:
        manifest = apk.read('assets/dashboard-assets.txt').decode().splitlines()
        expected = {'/' + p.relative_to(WEB).as_posix(): p for p in WEB.rglob('*')
                    if p.is_file() and p.suffix in {'.html', '.css', '.js', '.svg', '.png', '.txt'}}
        assert set(manifest) == set(expected)
        assert len(manifest) == len(expected)
        for url, source in expected.items():
            assert apk.read('assets/dashboard' + url) == source.read_bytes(), url
        assert not any('/api/' in name or name.endswith(('.token', '.env', '.keystore'))
                       for name in apk.namelist() if name.startswith('assets/'))


def test_all_startup_resources_resolve_inside_apk():
    class Resources(HTMLParser):
        def __init__(self):
            super().__init__()
            self.urls = []

        def handle_starttag(self, tag, attrs):
            attrs = dict(attrs)
            if tag in ('script', 'img'):
                self.urls.append(attrs.get('src', ''))
            elif tag == 'link':
                self.urls.append(attrs.get('href', ''))

    with ZipFile(APK) as apk:
        parser = Resources()
        parser.feed(apk.read('assets/dashboard/index.html').decode())
        assert parser.urls
        for url in parser.urls:
            if url.startswith('/'):
                assert 'assets/dashboard' + url in apk.namelist(), url
