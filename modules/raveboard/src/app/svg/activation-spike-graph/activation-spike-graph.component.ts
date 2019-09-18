import { Component, OnDestroy } from '@angular/core';
import { merge, Subscription } from "rxjs";

import { MockDataService } from "../../mock-data/mock-data.service";
import { ActivationUpdate, SpikeUpdate } from "../../model/model";
import { NodeType } from "../elements/node.component";
import { DomSanitizer } from "@angular/platform-browser";
import { SocketIOService } from "../../socketio/socketio.service";

export class NodeData {
    id: number;
    column: number;

    element: SpikeUpdate | ActivationUpdate;
    parents: Array<NodeData>;

    x: number;
    y: number;
    label: string;
    visible: boolean;
    transparent: boolean;
    nodeType: NodeType;

    constructor(element: SpikeUpdate | ActivationUpdate) {
        this.element = element;
        this.visible = true;
        this.transparent = false;
        this.parents = [];
        if (element.type === 'activation') {
            this.label = `${element.state} [${element.id}]`;
            this.id = NodeData.activationID(element.id);
            this.nodeType = NodeType.ACTIVATION;
        } else {
            this.label = `${element.signal} [${element.id}]`;
            this.id = NodeData.spikeID(element.id);
            this.nodeType = NodeType.SPIKE;
        }
    }

    static activationID(id: number): number {
        return id * 2;
    }
    static spikeID(id: number): number {
        return id * 2 + 1;
    }
}


@Component({
    selector: 'app-activation-spike-graph',
    template: `
        <svg [attr.viewBox]="'0 0 1000 1'" preserveAspectRatio="xMidYMid meet">
            <g class="moving-container" [style.transform]="getTransform()">

                <!-- connectors for every node -->
                <ng-container *ngFor="let node of allNodes.values()">
                    <ng-container *ngIf="node.visible">
                        <g connector *ngFor="let p of node.parents" [fromX]="p.x" [toX]="node.x" [fromY]="p.y" [toY]="node.y" 
                           [style.opacity]="hoveredNode && hoveredNode != node ? .1 : 1"></g>                        
                    </ng-container>
                </ng-container>

                <!-- all nodes on top of connectors -->
                <ng-container *ngFor="let node of allNodes.values()">
                    <g node *ngIf="node.visible"
                       (mouseenter)="hoverStart(node)" (mouseleave)="hoverEnd()"  
                       [x]="node.x" [y]="node.y" [label]="node.label" 
                       [nodeType]="node.nodeType" [nodeStatus]="node.element.status" 
                       [style.opacity]="node.transparent ? .2 : 1"></g>
                </ng-container>
                                
            </g>
        </svg>
        <div class="controls">
            <span class="percentage-label">{{(scale * 100).toFixed(0)}} %</span> 
            <button (click)="scaleDown()" class="round">-</button>
            <button (click)="scaleUp()" class="round">+</button>
            <button (click)="scale = 1">100%</button>
            <span class="separator">|</span>
            <button (click)="moveLeft()" class="round">&lt;</button>
            <button (click)="resetOffset()">Reset Offset</button>
            <button (click)="moveRight()" class="round">&gt;</button>
            <span class="separator">|</span>
            <button (click)="clear()">Clear Graph</button>
        </div>
    `,
    styleUrls: ['./activation-spike-graph.component.scss']
})
export class ActivationSpikeGraphComponent implements OnDestroy {

    private subscriptions: Subscription;

    // distance between two nodes (x-axis)
    nodeSpacingX = 180;
    ySpacing = 100;
    maxNodesPerColumn = 4;

    scale = 1;
    xOffset = 0;

    allNodes: Map<number, NodeData> = new Map();
    columns: Array<Set<NodeData>> = [new Set()];

    hoveredNode: NodeData;

    constructor(private mockDataService: MockDataService, private socketIoService: SocketIOService, private sanitizer: DomSanitizer)
    {

        // allow to mix real and mock data for development
        const mergedSpikes = merge(mockDataService.spikes, socketIoService.spikes);
        const mergedActs = merge(mockDataService.activations, socketIoService.activations);

        this.subscriptions = new Subscription();

        this.subscriptions.add(mergedSpikes.subscribe(spike => {

            const node = new NodeData(spike);
            this.allNodes.set(node.id, node);

            // find all spike parents
            let parentInLastCol = false;
            for (const parentID of spike.parents) {
                const nodeID = NodeData.spikeID(parentID);
                const parent = this.allNodes.get(nodeID);
                if (parent) {
                    if (parent.column === this.columns.length - 1) {
                        parentInLastCol = true;
                    }
                    node.parents.push(parent);
                }
            }
            const lastColumnFull = this.columns[this.columns.length - 1].size > this.maxNodesPerColumn;

            // create new column if necessary
            if (parentInLastCol || lastColumnFull) {
                this.columns.push(new Set());
            }

            // add new node to the last column and reposition nodes
            const lastColID = this.columns.length - 1;
            this.columns[lastColID].add(node);
            node.column = lastColID;

            this.repositionColumn(lastColID);  // reposition all nodes in that column

        }));

        this.subscriptions.add(mergedActs.subscribe(activation => {

            // create a new node for incoming activation
            const newNode = new NodeData(activation);

            // find all activation parents, determine lowest allowed column id
            let highestParentColumn = -1;
            for (const spikeMap of activation.spikes) {
                for (const spikeID of Object.values(spikeMap)) {
                    const nodeID = NodeData.spikeID(spikeID);
                    const parent = this.allNodes.get(nodeID);
                    if (parent) {
                        highestParentColumn = Math.max(highestParentColumn, parent.column);
                        newNode.parents.push(parent);
                    }
                }
            }
            // activations are only visible if they have parent spikes
            newNode.visible = newNode.parents.length > 0;

            // check if node with same activation id already exists,
            const prevNode = this.allNodes.get(newNode.id);

            let targetColumnID;
            if (prevNode && prevNode.column > highestParentColumn) {
                // replace prev with new node, in the same column
                targetColumnID = prevNode.column;

                // create new set to keep order, replace prev node
                const newColSet: Set<NodeData> = new Set();
                for (const colNode of this.columns[targetColumnID]) {
                    newColSet.add(colNode != prevNode ? colNode : newNode);
                }
                this.columns[targetColumnID] = newColSet;

            } else {
                // new must go to new column
                if (prevNode) { // delete prev
                    this.columns[prevNode.column].delete(prevNode);
                    this.repositionColumn(prevNode.column);  // reposition column where prev node was deleted
                }

                // need to insert new into last column
                if (highestParentColumn >= this.columns.length - 1
                    || this.columns[this.columns.length - 1].size > this.maxNodesPerColumn) {
                    // last column not ok (full or has parents)
                    this.columns.push(new Set());
                }
                targetColumnID = this.columns.length - 1;

                this.columns[targetColumnID].add(newNode);

            }

            newNode.column = targetColumnID;
            this.repositionColumn(targetColumnID);  // reposition column
            this.allNodes.set(newNode.id, newNode); // will also replace prevNode if it was there
        }));
    }

    private repositionColumn(targetColumnID: number) {
        let visibleNodes = 0;
        for (const node of this.columns[targetColumnID].values()) {
            if (node.visible) {
                visibleNodes++;
            }
        }

        // reposition column
        let y = - (visibleNodes - 1) / 2 * this.ySpacing;
        for (const node of this.columns[targetColumnID].values()) {
            if (!node.visible) {
                continue;
            }
            node.x = targetColumnID * this.nodeSpacingX;
            node.y = y;
            y += this.ySpacing;
        }

    }

    ngOnDestroy(): void {
        this.subscriptions.unsubscribe();
    }

    clear() {
        this.allNodes.clear();
        this.columns = [new Set()];
    }

    scaleUp() {
        if (this.scale < 4) {
            this.scale *= 1.1;
        }
    }

    scaleDown() {
        if (this.scale >= 0.3) {
            this.scale /= 1.1;
        }
    }

    moveLeft() {
        this.xOffset -= 300 / this.scale;
    }
    moveRight() {
        this.xOffset += 300 / this.scale;
    }
    resetOffset() {
        this.xOffset = 0;
    }

    getTransform() {
        const containerPosX = 800 - this.columns.length * this.nodeSpacingX - this.xOffset;
        const transform = `translateX(900px) scale(${this.scale.toFixed(2)}) translateX(-900px) translateX(${containerPosX}px)`;
        return this.sanitizer.bypassSecurityTrustStyle(transform);
    }

    hoverStart(hoveredNode: NodeData) {
        for (const node of this.allNodes.values()) {
            node.transparent = true;
        }
        hoveredNode.transparent = false;
        for (const parent of hoveredNode.parents) {
            parent.transparent = false;
        }


        this.hoveredNode = hoveredNode;
    }

    hoverEnd() {
        for (const node of this.allNodes.values()) {
            node.transparent = false;
        }
        this.hoveredNode = null;
    }

}
