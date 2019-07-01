import { Component } from '@angular/core';

@Component({
    selector: 'svg-experiments',
    template: `
        <svg>
            <!-- column 1 - column 2 -->
            <g curve-connector [fromX]="spacing" [fromY]="spacing" [toX]="spacing * 2" [toY]="spacing * 2"></g>
            <g curve-connector [fromX]="spacing" [fromY]="spacing" [toX]="spacing * 2" [toY]="spacing * 3"></g>
            <g curve-connector [fromX]="spacing" [fromY]="spacing * 2" [toX]="spacing * 2" [toY]="spacing * 2"></g>
            <g curve-connector [fromX]="spacing" [fromY]="spacing * 2" [toX]="spacing * 2" [toY]="spacing * 3"></g>
            <g curve-connector [fromX]="spacing" [fromY]="spacing * 3" [toX]="spacing * 2" [toY]="spacing"></g>

            <!-- column 2 - column 3 -->
            <g curve-connector [fromX]="spacing * 2" [fromY]="spacing" [toX]="spacing * 3" [toY]="spacing * 2"></g>
            <g curve-connector [fromX]="spacing * 2" [fromY]="spacing * 3" [toX]="spacing * 3" [toY]="spacing * 2"></g>

            <!-- column 1 - column 3 -->
            <g curve-connector [fromX]="spacing" [fromY]="spacing * 3" [toX]="spacing * 3" [toY]="spacing * 2"></g>

            <!-- nodes -->
            <g state-node [x]="spacing" [y]="spacing" [label]="'state 1'" [rectShape]="true"></g>
            <g state-node [x]="spacing" [y]="spacing * 2" [label]="'state 2'" [rectShape]="false"></g>
            <g state-node [x]="spacing" [y]="spacing * 3" [label]="'state 3'" [rectShape]="true"></g>

            <g signal-node [x]="spacing * 2" [y]="spacing" [label]="'hello'"></g>
            <g signal-node [x]="spacing * 2" [y]="spacing * 2" [label]="'world'"></g>
            <g signal-node [x]="spacing * 2" [y]="spacing * 3" [label]="'42!'"></g>

            <g property-node [x]="spacing * 3" [y]="spacing * 2" [label]="'prop'"></g>

        </svg>
    `,
    styleUrls: ['./svg-experiments.component.scss']
})
export class SvgExperimentsComponent {

    spacing = 100;

}
