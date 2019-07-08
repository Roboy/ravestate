import { Injectable } from '@angular/core';
import { Subject } from "rxjs";

@Injectable({
    providedIn: 'root'
})
export class MockDataService {

    dataStream: Subject<any>;

    constructor() {
        this.dataStream = new Subject();
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
