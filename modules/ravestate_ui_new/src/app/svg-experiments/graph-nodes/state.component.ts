import { Component, Input } from '@angular/core';

@Component({
    selector: '[state-node]',
    template: `
        <svg:circle *ngIf="!rectShape" class="state-node" [attr.cx]="x" [attr.cy]="y" r="30"/>
        <svg:rect *ngIf="rectShape" class="state-node" [attr.x]="x - width / 2" [attr.y]="y - height / 2" [attr.width]="width" [attr.height]="height" rx="5"/>
        
        <svg:text class="node-label" [attr.x]="x" [attr.y]="y+4" text-anchor="middle">{{label}}</svg:text>
    `,
    styleUrls: ['./node-styles.scss']
})
export class StateComponent {

    // rect params
    width = 80;
    height = 40;

    @Input() x: number = 0;
    @Input() y: number = 0;
    @Input() label: string = 'state';

    @Input() rectShape = false;

}
