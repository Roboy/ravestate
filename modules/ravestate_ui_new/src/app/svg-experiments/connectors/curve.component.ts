import { Component, Input } from '@angular/core';

@Component({
    selector: '[curve-connector]',
    template: `
        <svg:path class="connector-line" [attr.d]="pathD"/>
    `,
    styleUrls: ['./connector-styles.scss']
})
export class CurveComponent {

    @Input() fromX: number = 0;
    @Input() fromY: number = 0;

    @Input() toX: number = 0;
    @Input() toY: number = 0;

    get pathD(): string {
        // bezier control points
        const c1 = {
            x: Math.round((this.fromX + this.toX * 3) / 4),
            y: this.fromY
        };
        const c2 = {
            x: Math.round((this.fromX * 3 + this.toX) / 4),
            y: this.toY
        };
        return `M ${this.fromX} ${this.fromY} C ${c1.x} ${c1.y} ${c2.x} ${c2.y} ${this.toX} ${this.toY}`;
    }
}
