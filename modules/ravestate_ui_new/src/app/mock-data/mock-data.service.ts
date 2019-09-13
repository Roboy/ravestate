import { Injectable } from '@angular/core';
import { Observable, Subject } from 'rxjs';
import { filter, map } from 'rxjs/operators';
import { SpikeUpdate, ActivationUpdate } from "../model/model";

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

    dataStream: Subject<SpikeUpdate | ActivationUpdate>;

    activations: Observable<ActivationUpdate>;
    spikes: Observable<SpikeUpdate>;

    private randomIDCounter: number = 100;
    private mockMessageCounter: number = 0;

    private lastRandomSpikeID: number = -1;

    constructor() {
        this.dataStream = new Subject();
        this.activations = this.dataStream.pipe(filter(data => data.type === 'activation'), map(data => data as ActivationUpdate));
        this.spikes = this.dataStream.pipe(filter(data => data.type === 'spike'), map( data => data as SpikeUpdate));
    }

    public sendNextMockMessage() {
        if (this.mockMessageCounter < MOCK_MESSAGES.length) {
            this.dataStream.next(MOCK_MESSAGES[this.mockMessageCounter]);
            this.mockMessageCounter++;
        } else {
            this.sendSpike();
        }
    }

    resetMockData() {
        this.mockMessageCounter = 0;
    }

    public sendActivation() {
        this.dataStream.next({
            type: 'activation',
            id: this.randomIDCounter,
            state: 'random state',
            specificity: Math.floor(Math.random() * 100) / 100,
            status: 'ready',
            spikes: [{
                'random spike 1': this.randomIDCounter - 1,
                'random spike 2': this.randomIDCounter - 2,
                'random spike 4': this.randomIDCounter - 4,
                'another random spike': this.lastRandomSpikeID,
            }]
        });
        this.randomIDCounter++;
    }

    public sendSpike() {
        this.dataStream.next({
            type: 'spike',
            id: this.randomIDCounter,
            signal: 'random spike',
            parents: [this.randomIDCounter - 8, this.randomIDCounter - 14]
        });
        this.lastRandomSpikeID = this.randomIDCounter;
        this.randomIDCounter++;
    }

}
