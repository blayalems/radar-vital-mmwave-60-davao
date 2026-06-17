import { Pipe, type PipeTransform, inject } from "@angular/core"
import { I18nService } from "../services/i18n.service"

/**
 * Pure template helper. Pass `i18n.locale()` as the second argument so locale
 * changes invalidate Angular's pure-pipe cache without running every CD cycle.
 */
@Pipe({ name: "translate", standalone: true, pure: true })
export class TranslatePipe implements PipeTransform {
  private readonly i18n = inject(I18nService)

  transform(
    key: string,
    locale: string,
    params?: Record<string, string | number>,
  ): string {
    void locale
    return this.i18n.translate(key, params)
  }
}
