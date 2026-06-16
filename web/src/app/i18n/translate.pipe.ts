import { Pipe, type PipeTransform, inject } from "@angular/core"
import { I18nService } from "../services/i18n.service"

/**
 * Template helper for the runtime i18n layer.
 *
 * Usage:
 *   {{ 'nav.home' | translate }}
 *   {{ 'home.rerunCheck' | translate:{ label: check.label } }}
 *
 * The pipe is impure so an active-locale change is reflected immediately.
 * Locale switching is rare and the translated key count per template is small.
 */
@Pipe({ name: "translate", standalone: true, pure: false })
export class TranslatePipe implements PipeTransform {
  private readonly i18n = inject(I18nService)

  transform(key: string, params?: Record<string, string | number>): string {
    return this.i18n.translate(key, params)
  }
}
