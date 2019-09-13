
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