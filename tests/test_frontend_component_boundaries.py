from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
LIVE = ROOT / "web/src/app/components/live"


def test_live_overview_template_and_canvas_ownership_are_extracted():
    parent_html = (LIVE / "live.component.html").read_text(encoding="utf-8")
    parent_ts = (LIVE / "live.component.ts").read_text(encoding="utf-8")
    child_html = (
        LIVE / "tabs/live-overview-tab.component.html"
    ).read_text(encoding="utf-8")
    child_ts = (
        LIVE / "tabs/live-overview-tab.component.ts"
    ).read_text(encoding="utf-8")
    kpi_html = (
        LIVE / "tabs/live-kpi-strip.component.html"
    ).read_text(encoding="utf-8")
    kpi_ts = (
        LIVE / "tabs/live-kpi-strip.component.ts"
    ).read_text(encoding="utf-8")

    assert '<live-overview-tab [context]="overviewTabContext" />' in parent_html
    assert "kpi-metric-card" not in parent_html
    assert '<live-kpi-strip [context]="context()" />' in child_html
    assert "kpi-metric-card" not in child_html
    assert "kpi-metric-card" in kpi_html
    assert "LiveKpiStripViewModel" in kpi_ts
    assert "ChangeDetectionStrategy.OnPush" in kpi_ts
    assert "#targetCanvas" in child_html
    assert "#baCanvas" in child_html
    assert "@ViewChild('targetCanvas')" in child_ts
    assert "@ViewChild('baCanvas')" in child_ts
    assert "LiveOverviewTabViewModel" in child_ts
    assert "overviewTab?.targetCanvas" in parent_ts
    assert "overviewTab?.baCanvas" in parent_ts


def test_extracted_live_presentation_sources_stay_below_400_lines():
    presentation_files = [
        *LIVE.glob("tabs/*.component.ts"),
        *LIVE.glob("tabs/*.component.html"),
        *LIVE.glob("tabs/*.component.css"),
    ]
    oversized = {
        path.relative_to(ROOT).as_posix(): len(
            path.read_text(encoding="utf-8").splitlines()
        )
        for path in presentation_files
        if len(path.read_text(encoding="utf-8").splitlines()) > 400
    }
    assert oversized == {}
