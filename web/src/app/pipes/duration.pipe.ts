import { Pipe, PipeTransform } from '@angular/core';

@Pipe({ name: 'rvtDuration', standalone: true })
export class DurationPipe implements PipeTransform {
  transform(totalSeconds: number | null | undefined): string {
    if (totalSeconds == null || !Number.isFinite(totalSeconds) || totalSeconds < 0) return '--';
    const seconds = Math.round(totalSeconds);
    if (seconds < 60) return `${seconds} sec`;
    const minutes = Math.floor(seconds / 60);
    const remainingSeconds = seconds % 60;
    if (minutes < 60) {
      const minuteLabel = `${minutes} min`;
      return remainingSeconds ? `${minuteLabel} ${remainingSeconds} sec` : minuteLabel;
    }
    const hours = Math.floor(minutes / 60);
    const remainingMinutes = minutes % 60;
    const hourLabel = `${hours} hr`;
    return remainingMinutes ? `${hourLabel} ${remainingMinutes} min` : hourLabel;
  }
}
