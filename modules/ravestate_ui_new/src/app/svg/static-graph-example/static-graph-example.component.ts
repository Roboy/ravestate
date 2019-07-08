import { Component } from '@angular/core';
import { NodeType } from "../elements/node.component";

@Component({
    selector: 'svg-experiments',
    template: `
        <svg>
            <!-- column 1 - column 2 -->
            <g connector [fromX]="spacing" [fromY]="spacing" [toX]="spacing * 2" [toY]="spacing * 2"></g>
            <g connector [fromX]="spacing" [fromY]="spacing" [toX]="spacing * 2" [toY]="spacing * 3"></g>
            <g connector [fromX]="spacing" [fromY]="spacing * 2" [toX]="spacing * 2" [toY]="spacing * 2"></g>
            <g connector [fromX]="spacing" [fromY]="spacing * 2" [toX]="spacing * 2" [toY]="spacing * 3"></g>
            <g connector [fromX]="spacing" [fromY]="spacing * 3" [toX]="spacing * 2" [toY]="spacing"></g>

            <!-- column 2 - column 3 -->
            <g connector [fromX]="spacing * 2" [fromY]="spacing" [toX]="spacing * 3" [toY]="spacing * 2"></g>
            <g connector [fromX]="spacing * 2" [fromY]="spacing * 3" [toX]="spacing * 3" [toY]="spacing * 2"></g>

            <!-- column 1 - column 3 -->
            <g connector [fromX]="spacing" [fromY]="spacing * 3" [toX]="spacing * 3" [toY]="spacing * 2"></g>

            <!-- nodes -->
            <g node [x]="spacing" [y]="spacing" [label]="'activation'" [nodeType]="nodeType.ACTIVATION"></g>
            <g node [x]="spacing" [y]="spacing * 2" [label]="'spike 2'" [nodeType]="nodeType.SPIKE"></g>
            <g node [x]="spacing" [y]="spacing * 3" [label]="'spike 2'" [nodeType]="nodeType.SPIKE"></g>

            <g node [x]="spacing * 2" [y]="spacing" [label]="'hello'" [nodeType]="nodeType.ACTIVATION"></g>
            <g node [x]="spacing * 2" [y]="spacing * 2" [label]="'world'" [nodeType]="nodeType.SPIKE"></g>
            <g node [x]="spacing * 2" [y]="spacing * 3" [label]="'42!'" [nodeType]="nodeType.ACTIVATION"></g>

            <g node [x]="spacing * 3" [y]="spacing * 2" [label]="'prop'" [nodeType]="nodeType.ACTIVATION"></g>

        </svg>
    `,
    styleUrls: ['./static-graph-example.component.scss']
})
export class StaticGraphExampleComponent {

    nodeType = NodeType;

    spacing = 120;

}
