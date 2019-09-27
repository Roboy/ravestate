import { BrowserModule } from '@angular/platform-browser';
import { NgModule } from '@angular/core';

import { AppComponent } from './app.component';
import { NodeComponent } from './svg/elements/node.component';
import { ConnectorComponent } from './svg/elements/connector.component';
import { MockDataControllerComponent } from './model/mocks/mock-data-controller.component';
import { ActivationSpikeGraphComponent } from './svg/activation-spike-graph/activation-spike-graph.component';

@NgModule({
    declarations: [
        AppComponent,
        NodeComponent,
        ConnectorComponent,
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
