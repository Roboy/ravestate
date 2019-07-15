import { Component, OnDestroy } from '@angular/core';
import { MockDataService } from "../../mock-data/mock-data.service";
import { Subscription } from "rxjs";
import { NodeType } from "../elements/node.component";

@Component({
    selector: 'app-activation-spike-graph',
    template: `
        <svg>
            <g class="moving-container" [style.transform]="'translateX(' + wrapperPosX + 'px)'">
                <g node *ngFor="let node of nodes" [x]="node.x" [y]="node.y" [label]="node.label"
                   [nodeType]="node.type"></g>
            </g>
        </svg>
    `,
    styleUrls: ['./activation-spike-graph.component.scss']
})
export class ActivationSpikeGraphComponent implements OnDestroy {

    private dataSub: Subscription;

    nodes: Array<{x: number, y: number, label: string, type: NodeType}> = [];
    wrapperPosX = 0;

    constructor(private mockDataService: MockDataService) {
        this.dataSub = mockDataService.dataStream.subscribe(data => {
            const newNode = {
                x: 1000-this.wrapperPosX,
                y: 150 + Math.floor(Math.random() * 200),
                label: 'node ' + this.nodes.length,
                type: this.nodes.length % 2 == 0 ? NodeType.SPIKE : NodeType.ACTIVATION
            };
            this.nodes.push(newNode);
            this.wrapperPosX -= 100;
        });
    }

    ngOnDestroy(): void {
        this.dataSub.unsubscribe();
    }

}
