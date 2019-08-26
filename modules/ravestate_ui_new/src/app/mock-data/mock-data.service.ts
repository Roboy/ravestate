import { Injectable } from '@angular/core';
import { Observable, Subject } from 'rxjs';
import { filter, map } from 'rxjs/operators';

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


export interface SpikeUpdate {
    type: 'spike';

    /** (int) Identifies a unique spike - use to update view model. Can conflict with ActivationUpdate. */
    id: number;

    /** Signal represented by spike - use as caption. */
    signal: string;

    /** List of immediate parent spikes that caused this spike - draw connections pointing from parents to child spikes. */
    parents: Array<number>;
}

export interface ActivationUpdate {
    type: 'activation';

    /** (int) Identifies a unique activation - use to update view model. Can conflict with SpikeUpdate. */
    id: number;

    /** State represented by activation - use as caption. */
    state: string;

    /** (float) Specificity of state - visualise proportionally as relevance. */
    specificity: number;

    /**
     * Status of activation. Explanation:
     * - 'wait': Activation is waiting for additional spikes
     * - 'ready': Activation is waiting for permission to run
     * - 'run': Activation is executing
     */
    status: 'wait' | 'ready' | 'run';

    /**
     * Every map represents a disjunct conjunction of signal spike references as a map from signal name to spike ID.
     * A spike id of -1 indicates, that no spike for the given signal name is referenced yet.
     * An activation should only be displayed, if it references at least one spike.
     * Visualise referenced spikes as lines from the activation to the spike.
     */
    spikes: Array<{[signalName: string]: number}>;
}

const MOCK_MESSAGES: Array<SpikeUpdate | ActivationUpdate> = [
    // Message 0: A spike (id 0) is instantiated
    {
        type: 'spike',
        signal: 'rawio:in:changed',
        id: 0,
        parents: []
    },
    // Messages 1-3: Activations are introduced which reference the new spike
    {
        type: 'activation',
        id: 0,
        state: 'wildtalk',
        specificity: 0.2,
        status: 'ready',
        spikes: [{
            'rawio:in:changed': 0
        }]
    },
    {
        type: 'activation',
        id: 1,
        state: 'nlp',
        status: 'ready',
        specificity: 0.2,
        spikes: [{
            'rawio:in:changed': 0
        }]
    },
    {
        type: 'activation',
        id: 2,
        state: 'dinosaur-qa',
        status: 'wait',
        specificity: 1.2,
        spikes: [{
            'rawio:in:changed': 0,
            'nlp:is-question': -1,
            'visonio:is-dino': -1,
            'visonio:in:changed': -1
        }]
    },
    // Message 4: NLP state activation runs
    {
        type: 'activation',
        id: 1,
        state: 'nlp',
        specificity: 0.2,
        status: 'run',
        spikes: [{
            'rawio:in:changed': 0
        }]
    },
    // Message 5: NLP activation dereferences spikes
    {
        type: 'activation',
        id: 1,
        state: 'nlp',
        status: 'ready',
        specificity: 0.2,
        spikes: [{
            'rawio:in:changed': -1
        }]
    },
    // Message 6: nlp:is-question spike
    {
        type: 'spike',
        signal: 'nlp:is-question',
        id: 1,
        parents: [0]
    },
    // Message 7: dinosaur-qa refs. nlp:is-question
    {
        type: 'activation',
        id: 2,
        state: 'dinosaur-qa',
        status: 'wait',
        specificity: 1.2,
        spikes: [{
            'rawio:in:changed': 0,
            'nlp:is-question': 1,
            'visonio:is-dino': -1,
            'visonio:in:changed': -1
        }]
    },
    // Message 8: visionio:in:changed spike
    {
        type: 'spike',
        signal: 'visionio:in:changed',
        id: 2,
        parents: []
    },
    // Message 9-10: detect-objects, dinosaur-qa ref. new spike
    {
        type: 'activation',
        id: 2,
        state: 'dinosaur-qa',
        status: 'wait',
        specificity: 1.2,
        spikes: [{
            'rawio:in:changed': 0,
            'nlp:is-question': 1,
            'visonio:is-dino': -1,
            'visonio:in:changed': 2
        }]
    },
    {
        type: 'activation',
        id: 3,
        state: 'detect-objects',
        status: 'ready',
        specificity: 1.2,
        spikes: [{
            'visonio:in:changed': 2
        }]
    },
    // Message 11: detect-objects runs
    {
        type: 'activation',
        id: 3,
        state: 'detect-objects',
        status: 'run',
        specificity: 1.2,
        spikes: [{
            'visonio:in:changed': 2
        }]
    },
    // Message 12: detect-objects derefs. spikes
    {
        type: 'activation',
        id: 3,
        state: 'detect-objects',
        status: 'run',
        specificity: 1.2,
        spikes: [{
            'visonio:in:changed': -1
        }]
    },
    // Message 13-14: Follow-up spikes from visionio:in:changed
    {
        type: 'spike',
        signal: 'visionio:is-tree',
        id: 3,
        parents: [2]
    },
    {
        type: 'spike',
        signal: 'visionio:is-dino',
        id: 4,
        parents: [2]
    },
    // Message 15: dinosaur-qa refs. another spike
    {
        type: 'activation',
        id: 2,
        state: 'dinosaur-qa',
        status: 'ready',
        specificity: 1.2,
        spikes: [{
            'rawio:in:changed': 0,
            'nlp:is-question': 1,
            'visonio:is-dino': 4,
            'visonio:in:changed': 2
        }]
    },
    // Message 16-17: dinosaur-qa is run, wildtalk derefs.
    {
        type: 'activation',
        id: 0,
        state: 'wildtalk',
        specificity: 0.2,
        status: 'ready',
        spikes: [{
            'rawio:in:changed': -1
        }]
    },
    {
        type: 'activation',
        id: 2,
        state: 'dinosaur-qa',
        status: 'run',
        specificity: 1.2,
        spikes: [{
            'rawio:in:changed': 0,
            'nlp:is-question': 1,
            'visonio:is-dino': 4,
            'visonio:in:changed': 2
        }]
    },
];


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
                        'sample nonexistent spike': -1,
                        'sample spike 0': 0
                    },
                    id: 0,
                    name: 'sample activation 0',
                    specificity: 5
                }
            ],
            type: 'tick'
        });
    }

    public sendSpike() {
        this.dataStream.next({
            id: 0,
            name: 'sample spike 0',
            parent: 1,
            propertyValueAtCreation: 'whatever you want it to be',
            type: 'spike'
        });
    }

}
