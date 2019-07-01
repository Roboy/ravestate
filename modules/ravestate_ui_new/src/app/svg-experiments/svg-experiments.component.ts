import { Component } from '@angular/core';

@Component({
    selector: 'svg-experiments',
    template: `
        <svg>
            <!-- column 1 - column 2 -->
            <g curve-connector [fromX]="50" [fromY]="50" [toX]="150" [toY]="150"></g>
            <g curve-connector [fromX]="50" [fromY]="50" [toX]="150" [toY]="250"></g>
            <g curve-connector [fromX]="50" [fromY]="150" [toX]="150" [toY]="150"></g>
            <g curve-connector [fromX]="50" [fromY]="150" [toX]="150" [toY]="250"></g>
            <g curve-connector [fromX]="50" [fromY]="250" [toX]="150" [toY]="50"></g>
            
            <!-- column 2 - column 3 -->
            <g curve-connector [fromX]="150" [fromY]="50" [toX]="250" [toY]="150"></g>
            <g curve-connector [fromX]="150" [fromY]="250" [toX]="250" [toY]="150"></g>

            <!-- column 1 - column 3 -->
            <g curve-connector [fromX]="50" [fromY]="250" [toX]="250" [toY]="150"></g>

            <!-- nodes -->
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
