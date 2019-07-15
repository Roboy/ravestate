import { Component, OnDestroy } from '@angular/core';
import { MockDataService } from "../../mock-data/mock-data.service";
import { Subscription } from "rxjs";
import { NodeType } from "../elements/node.component";

@Component({
    selector: 'app-activation-spike-graph',
    template: `
        <svg>
            <g node *ngFor="let node of nodes" [x]="node.x" [y]="node.y" [label]="node.label" [nodeType]="node.type"></g>
        </svg>
    `,
    styleUrls: ['./activation-spike-graph.component.scss']
})
export class ActivationSpikeGraphComponent implements OnDestroy {

    nodes: Array<{x: number, y: number, label: string, type: NodeType}> = [];

    private dataSub: Subscription;

    constructor(private mockDataService: MockDataService) {
        this.dataSub = mockDataService.dataStream.subscribe(data => {
            console.log(data);

            for (const node of this.nodes) {
                node.x -= 100;
            }

            const newNode = {
                x: 500,
                y: 150 + Math.floor(Math.random() * 200),
                label: 'node ' + this.nodes.length,
                type: this.nodes.length % 2 == 0 ? NodeType.SPIKE : NodeType.ACTIVATION
            };

            this.nodes.push(newNode);

        });
    }

    ngOnDestroy(): void {
        this.dataSub.unsubscribe();
    }

}
