import { Pipe, type PipeTransform, inject } from "@angular/core"
import { I18nService } from "../services/i18n.service"

/**
 * Pure template helper. Pass `i18n.locale()` and `i18n.revision()` so locale
 * changes and late catalog registration both invalidate Angular's pure-pipe cache.
 */
@Pipe({ name: "translate", standalone: true, pure: true })
export class TranslatePipe implements PipeTransform {
  private readonly i18n = inject(I18nService)

  transform(
    key: string,
    locale: string,
    revision: number,
    params?: Record<string, string | number | null | undefined>,
  ): string {
    void locale
    void revision
    return this.i18n.translate(key, params)
  }
}
