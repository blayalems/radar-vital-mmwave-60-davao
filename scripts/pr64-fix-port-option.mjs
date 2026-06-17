import { readFile, writeFile } from 'node:fs/promises';
import path from 'node:path';
import url from 'node:url';

const __dirname = path.dirname(url.fileURLToPath(import.meta.url));
const root = path.resolve(__dirname, '..');
const templatePath = path.join(
  root,
  'web',
  'src',
  'app',
  'components',
  'home',
  'home.component.html',
);

const before = `              <mat-select [ngModel]="state.setup().radar_port" (ngModelChange)="updateSetup('radar_port', $event); onFormChange()" aria-labelledby="radarPortLabel">
                @for (port of radarPorts; track port) {
                <mat-option [value]="port">{{ port }}</mat-option>
                }
              </mat-select>`;

const after = `              <mat-select [ngModel]="state.setup().radar_port" (ngModelChange)="updateSetup('radar_port', $event); onFormChange()" aria-labelledby="radarPortLabel">
                @if (state.setup().radar_port && !radarPorts.includes(state.setup().radar_port)) {
                <mat-option [value]="state.setup().radar_port">{{ state.setup().radar_port }}</mat-option>
                }
                @for (port of radarPorts; track port) {
                <mat-option [value]="port">{{ port }}</mat-option>
                }
              </mat-select>`;

const source = await readFile(templatePath, 'utf8');
if (!source.includes(before)) {
  if (source.includes(after)) {
    console.log('Radar-port fallback option is already present.');
    process.exit(0);
  }
  throw new Error('Expected Radar Port select block was not found; refusing an unsafe patch.');
}

await writeFile(templatePath, source.replace(before, after), 'utf8');
console.log('Added a fallback option for the configured Radar Port.');
