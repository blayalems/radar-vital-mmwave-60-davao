import { Pipe, type PipeTransform, inject } from "@angular/core"
import { I18nService } from "../services/i18n.service"

/**
 * Template helper for the runtime i18n layer.
 *
 * Usage:
 *    'nav.home' | translate 
 *   {{ 'home.rerunCheck' | translate:{ label: check.label } }}
 *
 * Impure so it re-evaluates when the active locale changes (locale switching is
 * a rare event, and the key set per template is small, so the cost is
 * negligible). For pluralized strings call `I18nService.plural` directly.
 */
@Pipe({ name: "translate", standalone: true, pure: false })
export class TranslatePipe implements PipeTransform {
  private readonly i18n = inject(I18nService)

  transform(key: string, params?: Record<string, string | number>): string {
    return this.i18n.translate(key, params)
  }
}
