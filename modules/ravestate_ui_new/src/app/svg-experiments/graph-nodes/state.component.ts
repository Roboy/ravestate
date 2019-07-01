import { Component, Input } from '@angular/core';

@Component({
    selector: '[state-node]',
    template: `
        <svg:circle class="state-node" [attr.cx]="x" [attr.cy]="y" r="30"/>
        <svg:text class="node-label" [attr.x]="x" [attr.y]="y+4" text-anchor="middle">{{label}}</svg:text>
    `,
    styleUrls: ['./node-styles.scss']
})
export class StateComponent {

    @Input() x: number = 0;
    @Input() y: number = 0;
    @Input() label: string = 'state';

}
