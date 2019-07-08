import { BrowserModule } from '@angular/platform-browser';
import { NgModule } from '@angular/core';

import { AppComponent } from './app.component';
import { NodeComponent } from './svg/elements/node.component';
import { StaticGraphExampleComponent } from './svg/static-graph-example/static-graph-example.component';
import { ConnectorComponent } from './svg/elements/connector.component';
import { MockDataControllerComponent } from './mock-data/mock-data-controller/mock-data-controller.component';
import { ActivationSpikeGraphComponent } from './svg/activation-spike-graph/activation-spike-graph.component';

@NgModule({
    declarations: [
        AppComponent,
        NodeComponent,
        ConnectorComponent,
        StaticGraphExampleComponent,
        MockDataControllerComponent,
        ActivationSpikeGraphComponent
    ],
    imports: [
        BrowserModule
    ],
    providers: [],
    bootstrap: [AppComponent]
})
export class AppModule {
}
