import { BrowserModule } from '@angular/platform-browser';
import { NgModule } from '@angular/core';

import { AppComponent } from './app.component';
import { StateComponent } from './svg-experiments/graph-nodes/state.component';
import { PropertyComponent } from './svg-experiments/graph-nodes/property.component';
import { SignalComponent } from './svg-experiments/graph-nodes/signal.component';
import { SvgExperimentsComponent } from './svg-experiments/svg-experiments.component';
import { CurveComponent } from './svg-experiments/connectors/curve.component';

@NgModule({
    declarations: [
        AppComponent,
        StateComponent,
        PropertyComponent,
        SignalComponent,
        SvgExperimentsComponent,
        CurveComponent
    ],
    imports: [
        BrowserModule
    ],
    providers: [],
    bootstrap: [AppComponent]
})
export class AppModule {
}
