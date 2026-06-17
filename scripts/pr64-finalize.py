from __future__ import annotations

from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def read(relative: str) -> str:
    return (ROOT / relative).read_text(encoding="utf-8")


def write(relative: str, content: str) -> None:
    (ROOT / relative).write_text(content, encoding="utf-8")


def replace_once(relative: str, old: str, new: str) -> None:
    text = read(relative)
    count = text.count(old)
    if count != 1:
        raise RuntimeError(f"{relative}: expected one exact match, found {count}: {old!r}")
    write(relative, text.replace(old, new, 1))


# 1) Keep static theme corrections at their actual source and reserve the
# external runtime sheet for inline Material You overrides only.
replace_once(
    "web/src/styles/rvt-redesign-tokens.css",
    "--rv-ink: #e0e8f2; --rv-ink-2: #d3e4ff; --rv-muted: #93a4b8; --rv-dim: #9fb0c4;",
    "--rv-ink: #e0e8f2; --rv-ink-2: #d3e4ff; --rv-muted: #94a5b9; --rv-dim: #9fb0c4;",
)

runtime_css = read("assets/fonts/rvt-runtime-accessibility.css")
if "html[data-theme=\"dark\"][data-palette=\"azure\"]" not in runtime_css:
    raise RuntimeError("runtime accessibility stylesheet no longer matches the expected PR64 source")
write(
    "assets/fonts/rvt-runtime-accessibility.css",
    """/* Runtime-only accessibility mapping for Material You inline color schemes.\n"
    "   Static palette, Night, and High Contrast tokens live in\n"
    "   web/src/styles/rvt-redesign-tokens.css as the single source of truth. */\n"
    "html[data-dynamic-color=\"1\"][data-theme=\"light\"],\n"
    "html[data-dynamic-color=\"1\"][data-theme=\"dark\"],\n"
    "html[data-dynamic-color=\"1\"][data-theme=\"night\"] {\n"
    "  --rv-muted: var(--md-sys-color-on-surface-variant, #c3c7d0);\n"
    "  --rv-dim: var(--md-sys-color-on-surface-variant, #c3c7d0);\n"
    "}\n""",
)

# 2) Make the stable external font/runtime stylesheet available under ng serve
# at the same URL used by production, PWA, APK, EXE, and the monolith.
replace_once(
    "web/angular.json",
    '''              {
                "glob": "**/*",
                "input": "../assets",
                "output": "assets"
              }
''',
    '''              {
                "glob": "**/*",
                "input": "../assets",
                "output": "assets"
              },
              {
                "glob": "**/*",
                "input": "../assets/fonts",
                "output": "fonts"
              }
''',
)
replace_once(
    "web/src/index.html",
    '  <base href="./">\n</head>',
    '  <base href="./">\n  <link rel="stylesheet" href="./fonts/rvt-fonts.css" data-rvt-runtime-fonts>\n</head>',
)

# 3) Expose the selected duration and polling state to assistive technology.
home_path = "web/src/app/components/home/home.component.html"
for seconds in (30, 60, 300, 480, 1200):
    replace_once(
        home_path,
        f'<button mat-flat-button type="button" class="mat-tonal-button duration-chip" [class.active]="selectedDuration === {seconds}" (click)="selectDuration({seconds})">',
        f'<button mat-flat-button type="button" class="mat-tonal-button duration-chip" [class.active]="selectedDuration === {seconds}" [attr.aria-pressed]="selectedDuration === {seconds}" (click)="selectDuration({seconds})">',
    )

# 4) Activate the i18n infrastructure in the touched Home controls.
replace_once(
    home_path,
    '<label class="field-label" id="recDurationLabel">Recording Duration <span class="label-hint">how long to record</span></label>',
    '<label class="field-label" id="recDurationLabel">{{ \'home.recordingDuration\' | translate:i18n.locale():i18n.revision() }} <span class="label-hint">how long to record</span></label>',
)
replace_once(
    home_path,
    '<label class="field-label" id="radarPortLabel">Radar Port</label>',
    '<label class="field-label" id="radarPortLabel">{{ \'home.radarPort\' | translate:i18n.locale():i18n.revision() }}</label>',
)
replace_once(
    home_path,
    'class="mat-tonal-button icon-action-btn" aria-label="Scan for radar serial ports"',
    'class="mat-tonal-button icon-action-btn" [attr.aria-label]="\'home.scanPorts\' | translate:i18n.locale():i18n.revision()"',
)
replace_once(
    home_path,
    '<label class="field-label" id="bleAddressLabel">BLE Oximeter Address</label>',
    '<label class="field-label" id="bleAddressLabel">{{ \'home.bleAddress\' | translate:i18n.locale():i18n.revision() }}</label>',
)
replace_once(
    home_path,
    '[attr.aria-label]="\'Re-run check: \' + check.label"',
    '[attr.aria-label]="\'home.rerunCheck\' | translate:i18n.locale():i18n.revision():{ label: check.label }"',
)

home_ts = "web/src/app/components/home/home.component.ts"
replace_once(
    home_ts,
    "import { ServerLifecycleService } from '../../services/server-lifecycle.service';\n",
    "import { ServerLifecycleService } from '../../services/server-lifecycle.service';\nimport { I18nService } from '../../services/i18n.service';\nimport { TranslatePipe } from '../../i18n/translate.pipe';\n",
)
replace_once(
    home_ts,
    "    MatChipsModule,\n    MatSnackBarModule\n",
    "    MatChipsModule,\n    MatSnackBarModule,\n    TranslatePipe\n",
)
replace_once(
    home_ts,
    "  protected readonly serverLifecycle = inject(ServerLifecycleService);\n",
    "  protected readonly serverLifecycle = inject(ServerLifecycleService);\n  protected readonly i18n = inject(I18nService);\n",
)

# 5) Make Topbar labels translated and state-specific.
topbar_path = "web/src/app/components/topbar/topbar.component.html"
replace_once(
    topbar_path,
    'title="Simple view" aria-label="Simple view"',
    '[attr.title]="\'topbar.simpleView\' | translate:i18n.locale():i18n.revision()" [attr.aria-label]="\'topbar.simpleView\' | translate:i18n.locale():i18n.revision()"',
)
replace_once(
    topbar_path,
    'title="Advanced view" aria-label="Advanced view"',
    '[attr.title]="\'topbar.advancedView\' | translate:i18n.locale():i18n.revision()" [attr.aria-label]="\'topbar.advancedView\' | translate:i18n.locale():i18n.revision()"',
)
replace_once(
    topbar_path,
    'title="Comfortable" aria-label="Comfortable density"',
    '[attr.title]="\'topbar.comfortableDensity\' | translate:i18n.locale():i18n.revision()" [attr.aria-label]="\'topbar.comfortableDensity\' | translate:i18n.locale():i18n.revision()"',
)
replace_once(
    topbar_path,
    'title="Compact" aria-label="Compact density"',
    '[attr.title]="\'topbar.compactDensity\' | translate:i18n.locale():i18n.revision()" [attr.aria-label]="\'topbar.compactDensity\' | translate:i18n.locale():i18n.revision()"',
)
replace_once(
    topbar_path,
    '<button mat-icon-button class="ic-btn icon-btn tb-pause" [class.active]="state.paused()" (click)="togglePause()" type="button" title="Pause / resume live polling" aria-label="Pause or resume live polling">',
    '<button mat-icon-button class="ic-btn icon-btn tb-pause" [class.active]="state.paused()" (click)="togglePause()" type="button" [attr.title]="(state.paused() ? \'topbar.resumePolling\' : \'topbar.pausePolling\') | translate:i18n.locale():i18n.revision()" [attr.aria-label]="(state.paused() ? \'topbar.resumePolling\' : \'topbar.pausePolling\') | translate:i18n.locale():i18n.revision()" [attr.aria-pressed]="state.paused()">',
)
replace_once(
    topbar_path,
    '<button mat-menu-item type="button" (click)="togglePause()"><mat-icon>{{ state.paused() ? \'play_arrow\' : \'pause\' }}</mat-icon><span>Pause or resume</span></button>',
    '<button mat-menu-item type="button" (click)="togglePause()"><mat-icon>{{ state.paused() ? \'play_arrow\' : \'pause\' }}</mat-icon><span>{{ (state.paused() ? \'topbar.resumePolling\' : \'topbar.pausePolling\') | translate:i18n.locale():i18n.revision() }}</span></button>',
)

topbar_ts = "web/src/app/components/topbar/topbar.component.ts"
replace_once(
    topbar_ts,
    "import { SwitchOperatorDialogComponent } from '../switch-operator-dialog/switch-operator-dialog.component';\n",
    "import { SwitchOperatorDialogComponent } from '../switch-operator-dialog/switch-operator-dialog.component';\nimport { I18nService } from '../../services/i18n.service';\nimport { TranslatePipe } from '../../i18n/translate.pipe';\n",
)
replace_once(
    topbar_ts,
    "    MatMenuModule,\n    MatToolbarModule\n",
    "    MatMenuModule,\n    MatToolbarModule,\n    TranslatePipe\n",
)
replace_once(
    topbar_ts,
    "  protected readonly auth = inject(AuthService);\n",
    "  protected readonly auth = inject(AuthService);\n  protected readonly i18n = inject(I18nService);\n",
)

# 6) Make pure-pipe invalidation include catalog updates, not only locale changes.
replace_once(
    "web/src/app/services/i18n.service.ts",
    "  private readonly catalogRevision = signal(0)\n\n  readonly activeCatalog",
    "  private readonly catalogRevision = signal(0)\n  readonly revision = this.catalogRevision.asReadonly()\n\n  readonly activeCatalog",
)
replace_once(
    "web/src/app/i18n/translate.pipe.ts",
    " * Pure template helper. Pass `i18n.locale()` as the second argument so locale\n * changes invalidate Angular's pure-pipe cache without running every CD cycle.\n",
    " * Pure template helper. Pass `i18n.locale()` and `i18n.revision()` so locale\n * changes and late catalog registration both invalidate Angular's pure-pipe cache.\n",
)
replace_once(
    "web/src/app/i18n/translate.pipe.ts",
    "    locale: string,\n    params?: Record<string, string | number>,\n  ): string {\n    void locale\n",
    "    locale: string,\n    revision: number,\n    params?: Record<string, string | number>,\n  ): string {\n    void locale\n    void revision\n",
)

catalog_path = "web/src/app/services/i18n.messages.en.ts"
replace_once(
    catalog_path,
    '  "topbar.togglePolling": "Pause or resume live polling",\n',
    '  "topbar.togglePolling": "Pause or resume live polling",\n  "topbar.pausePolling": "Pause live polling",\n  "topbar.resumePolling": "Resume live polling",\n',
)

pipe_spec = "web/src/app/i18n/translate.pipe.spec.ts"
replace_once(pipe_spec, 'pipe.transform("nav.home", "en")', 'pipe.transform("nav.home", "en", 0)')
replace_once(
    pipe_spec,
    'pipe.transform("home.rerunCheck", "en", { label: "Radar link" })',
    'pipe.transform("home.rerunCheck", "en", 0, { label: "Radar link" })',
)
replace_once(
    pipe_spec,
    '''  it("interpolates parameters without changing the pure-pipe contract", () => {
    const pipe = TestBed.runInInjectionContext(() => new TranslatePipe())
    expect(
      pipe.transform("home.rerunCheck", "en", 0, { label: "Radar link" }),
    ).toBe("Re-run check: Radar link")
  })
''',
    '''  it("interpolates parameters without changing the pure-pipe contract", () => {
    const pipe = TestBed.runInInjectionContext(() => new TranslatePipe())
    expect(
      pipe.transform("home.rerunCheck", "en", 0, { label: "Radar link" }),
    ).toBe("Re-run check: Radar link")
  })

  it("uses the catalog revision as a pure-pipe invalidation key", () => {
    const i18n = TestBed.inject(I18nService)
    const pipe = TestBed.runInInjectionContext(() => new TranslatePipe())
    i18n.registerCatalog("fil", { "nav.home": "Tahanan" })
    i18n.setLocale("fil")
    expect(
      pipe.transform("nav.home", i18n.locale(), i18n.revision()),
    ).toBe("Tahanan")

    i18n.registerCatalog("fil", { "nav.home": "Pangunahing pahina" })
    expect(
      pipe.transform("nav.home", i18n.locale(), i18n.revision()),
    ).toBe("Pangunahing pahina")
  })
''',
)

# 7) Document the two pure-pipe cache keys.
replace_once(
    "docs/i18n.md",
    "Pass the locale signal value as the pure-pipe cache key:\n",
    "Pass the locale and catalog-revision signal values as pure-pipe cache keys:\n",
)
replace_once(
    "docs/i18n.md",
    "<h2>{{ 'home.systemChecks' | translate:i18n.locale() }}</h2>\n\n<button\n  [attr.aria-label]=\"'home.rerunCheck' | translate:i18n.locale():{ label: check.label }\">",
    "<h2>{{ 'home.systemChecks' | translate:i18n.locale():i18n.revision() }}</h2>\n\n<button\n  [attr.aria-label]=\"'home.rerunCheck' | translate:i18n.locale():i18n.revision():{ label: check.label }\">",
)
replace_once(
    "docs/i18n.md",
    "This keeps locale changes reactive without evaluating translations on every unrelated change-detection cycle.\n",
    "This keeps locale and late catalog-registration changes reactive without evaluating translations on every unrelated change-detection cycle.\n",
)

# 8) Test the actual generated Material You scheme for representative seeds.
dynamic_spec = '''import { TestBed } from "@angular/core/testing"
import { afterEach, beforeEach, describe, expect, it } from "vitest"
import { WCAG_AA_NORMAL, contrastRatio } from "../a11y/contrast-core.mjs"
import { DynamicColorService } from "./dynamic-color.service"

const SEEDS = [
  "#0061a4",
  "#ff0000",
  "#00ff00",
  "#0000ff",
  "#00ffff",
  "#ff00ff",
]

const THEMES = ["light", "dark", "night"] as const
const BACKGROUND_TOKENS = [
  "--md-sys-color-surface",
  "--md-sys-color-surface-container-lowest",
  "--md-sys-color-surface-container-low",
  "--md-sys-color-surface-container",
  "--md-sys-color-surface-container-high",
  "--md-sys-color-surface-container-highest",
  "--md-sys-color-secondary-container",
]

describe("DynamicColorService low-emphasis contrast", () => {
  let service: DynamicColorService

  beforeEach(() => {
    TestBed.resetTestingModule()
    localStorage.clear()
    const html = document.documentElement
    html.removeAttribute("style")
    delete html.dataset["dynamicColor"]
    delete html.dataset["palette"]
    TestBed.configureTestingModule({ providers: [DynamicColorService] })
    service = TestBed.inject(DynamicColorService)
  })

  afterEach(() => {
    const html = document.documentElement
    html.removeAttribute("style")
    delete html.dataset["dynamicColor"]
    delete html.dataset["theme"]
    TestBed.resetTestingModule()
  })

  it("keeps generated on-surface-variant text above WCAG AA on every generated surface", () => {
    const html = document.documentElement

    for (const theme of THEMES) {
      html.dataset["theme"] = theme
      for (const seed of SEEDS) {
        service.setSourceColor(seed)
        service.setEnabled(true)
        service.reapply()

        expect(html.dataset["dynamicColor"]).toBe("1")
        const foreground = html.style
          .getPropertyValue("--md-sys-color-on-surface-variant")
          .trim()
        expect(foreground).toMatch(/^#[0-9a-f]{6}$/i)

        for (const token of BACKGROUND_TOKENS) {
          const background = html.style.getPropertyValue(token).trim()
          expect(background).toMatch(/^#[0-9a-f]{6}$/i)
          expect(
            contrastRatio(foreground, background),
            `${theme} ${seed}: ${foreground} on ${token} ${background}`,
          ).toBeGreaterThanOrEqual(WCAG_AA_NORMAL)
        }
      }
    }
  })
})
'''
write("web/src/app/services/dynamic-color.service.spec.ts", dynamic_spec)

# 9) Make the static checker verify that the runtime mapping follows the same
# generated on-surface-variant token tested above.
replace_once(
    "scripts/check-contrast-tokens.mjs",
    "const overrideCss = await readFile(OVERRIDE_CSS_PATH, 'utf8');\n\nconst commonSurfaceTokens",
    """const overrideCss = await readFile(OVERRIDE_CSS_PATH, 'utf8');

const dynamicSelectors = [
  'html[data-dynamic-color="1"][data-theme="light"]',
  'html[data-dynamic-color="1"][data-theme="dark"]',
  'html[data-dynamic-color="1"][data-theme="night"]',
];
const expectedDynamicText =
  'var(--md-sys-color-on-surface-variant, #c3c7d0)';
for (const selector of dynamicSelectors) {
  const declarations = parseDeclarations(overrideCss, selector);
  for (const token of ['--rv-muted', '--rv-dim']) {
    if (declarations.get(token) !== expectedDynamicText) {
      throw new Error(
        `${selector} must map ${token} to the generated on-surface-variant token`,
      );
    }
  }
}

const commonSurfaceTokens""",
)
replace_once(
    "scripts/check-contrast-tokens.mjs",
    "      { css: baseCss, selector: 'html[data-theme=\"night\"]' },\n      { css: redesignCss, selector: 'html[data-theme=\"night\"]' },\n      { css: overrideCss, selector: 'html[data-theme=\"night\"]' },\n",
    "      { css: baseCss, selector: 'html[data-theme=\"night\"]' },\n      { css: redesignCss, selector: 'html[data-theme=\"night\"]' },\n",
)
replace_once(
    "scripts/check-contrast-tokens.mjs",
    "      { css: baseCss, selector: 'html[data-theme=\"hc\"]' },\n      { css: redesignCss, selector: 'html[data-theme=\"hc\"]' },\n      { css: overrideCss, selector: 'html[data-theme=\"hc\"]' },\n",
    "      { css: baseCss, selector: 'html[data-theme=\"hc\"]' },\n      { css: redesignCss, selector: 'html[data-theme=\"hc\"]' },\n",
)

# 10) Lock the new semantics and dev parity into source-integrity validation.
replace_once(
    "scripts/check-source-integrity.mjs",
    "  'web/angular.json',\n",
    "  'web/angular.json',\n  'web/src/index.html',\n",
)
replace_once(
    "scripts/check-source-integrity.mjs",
    "  'web/src/app/services/i18n.messages.en.ts',\n",
    "  'web/src/app/services/dynamic-color.service.spec.ts',\n  'web/src/app/services/i18n.messages.en.ts',\n",
)
replace_once(
    "scripts/check-source-integrity.mjs",
    "    /\\{\\{\\s*session\\.started_at\\s*\\|\\s*date\\s*:\\s*['\"]shortTime['\"]\\s*\\}\\}/,\n",
    "    /\\{\\{\\s*session\\.started_at\\s*\\|\\s*date\\s*:\\s*['\"]shortTime['\"]\\s*\\}\\}/,\n    /\\[attr\\.aria-pressed\\]=\"selectedDuration === 30\"/,\n    /translate:i18n\\.locale\\(\\):i18n\\.revision\\(\\)/,\n",
)
replace_once(
    "scripts/check-source-integrity.mjs",
    "    /\\{\\{\\s*state\\.paused\\(\\)\\s*\\?\\s*['\"]play_arrow['\"]\\s*:\\s*['\"]pause['\"]\\s*\\}\\}/,\n",
    "    /\\{\\{\\s*state\\.paused\\(\\)\\s*\\?\\s*['\"]play_arrow['\"]\\s*:\\s*['\"]pause['\"]\\s*\\}\\}/,\n    /\\[attr\\.aria-pressed\\]=\"state\\.paused\\(\\)\"/,\n    /topbar\\.resumePolling/,\n    /translate:i18n\\.locale\\(\\):i18n\\.revision\\(\\)/,\n",
)

# 11) Ensure changed external assets are unmistakably refreshed by the PWA.
replace_once("assets/sw.js", "const CACHE = 'rvt-shell-v12.0.5';", "const CACHE = 'rvt-shell-v12.0.6';")

print("PR64 follow-up source fixes applied successfully.")
