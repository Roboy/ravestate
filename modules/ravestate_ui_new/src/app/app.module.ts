import { BrowserModule } from '@angular/platform-browser';
import { NgModule } from '@angular/core';

import { AppComponent } from './app.component';
import { NodeComponent } from './svg-experiments/elements/node.component';
import { SvgExperimentsComponent } from './svg-experiments/svg-experiments.component';
import { ConnectorComponent } from './svg-experiments/elements/connector.component';
import { MockDataControllerComponent } from './mock-data/mock-data-controller/mock-data-controller.component';

@NgModule({
    declarations: [
        AppComponent,
        NodeComponent,
        ConnectorComponent,
        SvgExperimentsComponent,
        MockDataControllerComponent
    ],
    imports: [
        BrowserModule
    ],
    providers: [],
    bootstrap: [AppComponent]
})
export class AppModule {
}
