import { ChangeDetectionStrategy, Component, input } from '@angular/core';
import { MatCardModule } from '@angular/material/card';
import { MatIconModule } from '@angular/material/icon';

import {
  PRODUCT_NAME,
  AUTHORS,
  PROGRAM,
  UNIVERSITY,
  GITHUB_REPO_URL,
  copyrightLine,
} from '../../services/app-meta';

@Component({
  selector: 'app-about-card',
  standalone: true,
  imports: [MatCardModule, MatIconModule],
  templateUrl: './about-card.component.html',
  styleUrl: './about-card.component.css',
  changeDetection: ChangeDetectionStrategy.OnPush,
})
export class AboutCardComponent {
  /** Product version string — passed in by the settings host so the card stays decoupled from any version service. */
  readonly version = input<string>('');

  protected readonly productName = PRODUCT_NAME;
  protected readonly authors = AUTHORS;
  protected readonly program = PROGRAM;
  protected readonly university = UNIVERSITY;
  protected readonly githubRepoUrl = GITHUB_REPO_URL;
  protected readonly copyrightText = copyrightLine();

  protected readonly termsUrl = `${GITHUB_REPO_URL}/blob/main/TERMS.md`;
  protected readonly privacyUrl = `${GITHUB_REPO_URL}/blob/main/PRIVACY.md`;
  protected readonly licenseUrl = `${GITHUB_REPO_URL}/blob/main/LICENSE`;
}
