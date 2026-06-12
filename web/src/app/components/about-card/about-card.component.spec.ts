import { ComponentFixture, TestBed } from '@angular/core/testing';
import { AboutCardComponent } from './about-card.component';
import { AUTHORS, UNIVERSITY } from '../../services/app-meta';

describe('AboutCardComponent', () => {
  let fixture: ComponentFixture<AboutCardComponent>;
  let element: HTMLElement;

  beforeEach(async () => {
    await TestBed.configureTestingModule({
      imports: [AboutCardComponent],
    }).compileComponents();

    fixture = TestBed.createComponent(AboutCardComponent);
    fixture.detectChanges();
    element = fixture.nativeElement as HTMLElement;
  });

  it('should create the component', () => {
    expect(fixture.componentInstance).toBeTruthy();
  });

  it('renders all three authors', () => {
    const text = element.textContent ?? '';
    for (const author of AUTHORS) {
      expect(text).toContain(author);
    }
  });

  it('renders the university name', () => {
    expect(element.textContent).toContain(UNIVERSITY);
  });

  it('copyright line contains the current year', () => {
    const year = new Date().getFullYear().toString();
    expect(element.textContent).toContain(year);
  });

  it('contains a Terms link pointing to TERMS.md', () => {
    const link = element.querySelector<HTMLAnchorElement>('a[href*="TERMS.md"]');
    expect(link).toBeTruthy();
    expect(link?.getAttribute('target')).toBe('_blank');
    expect(link?.getAttribute('rel')).toContain('noopener');
  });

  it('contains a Privacy link pointing to PRIVACY.md', () => {
    const link = element.querySelector<HTMLAnchorElement>('a[href*="PRIVACY.md"]');
    expect(link).toBeTruthy();
    expect(link?.getAttribute('target')).toBe('_blank');
    expect(link?.getAttribute('rel')).toContain('noopener');
  });

  it('contains a License link pointing to LICENSE', () => {
    const link = element.querySelector<HTMLAnchorElement>('a[href*="LICENSE"]');
    expect(link).toBeTruthy();
    expect(link?.getAttribute('target')).toBe('_blank');
    expect(link?.getAttribute('rel')).toContain('noopener');
  });

  it('contains a GitHub repo link', () => {
    const link = element.querySelector<HTMLAnchorElement>('a[href*="github.com"]');
    expect(link).toBeTruthy();
    expect(link?.getAttribute('target')).toBe('_blank');
    expect(link?.getAttribute('rel')).toContain('noopener');
  });

  it('renders version when provided via input', async () => {
    fixture.componentRef.setInput('version', '1.2.3');
    fixture.detectChanges();
    await fixture.whenStable();
    expect(element.textContent).toContain('1.2.3');
  });
});
