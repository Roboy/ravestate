import { Component } from '@angular/core';

@Component({
    selector: 'svg-experiments',
    template: `
        <svg>
            <g state-node [x]="50" [y]="50" [label]="'state 1'"></g>
            <g state-node [x]="50" [y]="150" [label]="'state 2'"></g>
            <g state-node [x]="50" [y]="250" [label]="'state 3'"></g>
            
            <g signal-node [x]="150" [y]="50" [label]="'hello'"></g>
            <g signal-node [x]="150" [y]="150" [label]="'world'"></g>
            <g signal-node [x]="150" [y]="250" [label]="'42!'"></g>
            
            <g property-node [x]="250" [y]="150" [label]="'prop'"></g>
        </svg>
    `,
    styleUrls: ['./svg-experiments.component.scss']
})
export class SvgExperimentsComponent {

    constructor() { }

}
