import { Injectable } from '@angular/core';
import { Observable, Subject } from "rxjs";
import { filter, map } from "rxjs/operators";

export interface ActivationsTick {
    type: 'tick';
    activations: Array<{
        activated: boolean,
        constrains: {[name: string]: number},
        id: number,
        name: string,
        specificity: number
    }>;
}

export interface Spike {
    type: 'spike';
    id: number;
    name: string;
    parent: number;
    propertyValueAtCreation: any;
}


@Injectable({
    providedIn: 'root'
})
export class MockDataService {

    dataStream: Subject<{type: 'tick' | 'spike', [others: string]: any}>;

    ticks: Observable<ActivationsTick>;
    spikes: Observable<Spike>;

    constructor() {
        this.dataStream = new Subject();
        this.ticks = this.dataStream.pipe(filter(data => data.type === 'tick'), map(data => data as ActivationsTick));
        this.spikes = this.dataStream.pipe(filter(data => data.type === 'spike'), map( data => data as Spike));
    }

    public sendActivations() {
        this.dataStream.next({
            activations: [
                {
                    activated: false,
                    constraints: {
                        "sample nonexistent spike": -1,
                        "sample spike 0": 0
                    },
                    id: 0,
                    name: "sample activation 0",
                    specificity: 5
                }
            ],
            type: "tick"
        });
    }

    public sendSpike() {
        this.dataStream.next({
            id: 0,
            name: "sample spike 0",
            parent: 1,
            propertyValueAtCreation: "whatever you want it to be",
            type: "spike"
        });
    }

}
