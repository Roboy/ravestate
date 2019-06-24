import { Component } from '@angular/core';

@Component({
    selector: 'svg-experiments',
    template: `
        <svg>
            <g signal-node [x]="70" [y]="40" [label]="'hello'"></g>
            <g signal-node [x]="70" [y]="120" [label]="'world'"></g>
            <g signal-node [x]="70" [y]="200"></g>
            <g state-node></g>
            <g property-node></g>
        </svg>
    `,
    styleUrls: ['./svg-experiments.component.scss']
})
export class SvgExperimentsComponent {

    constructor() { }

}
