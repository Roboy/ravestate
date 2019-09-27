export interface SpikeUpdate {
    type: 'spike';

    /** (int) Identifies a unique spike - use to update view model. Can conflict with ActivationUpdate. */
    id: number;

    /** Signal represented by spike - use as caption. */
    signal: string;

    /** List of immediate parent spikes that caused this spike - draw connections pointing from parents to child spikes. */
    parents: Array<number>;
}
