# Ravestate UI

Roboy ravestate UI module. 

## Setup, Development and Building

This project is built with Angular 8 and TypeScript. 

Initial setup:
- install Angular CLI globally by running `npm install -g @angular/cli` (don't forget to add `<home>/.npm-global/bin` to your path)
- Install project dependencies by running `npm install` in the root folder (= folder with `package.json`) 
 
To start development: 
- run `ng serve` in the root folder to start a dev server
- Navigate to `http://localhost:4200/` in the browser to open the UI
- The app will automatically reload if you change any of the source files

To build a production version:
- run `ng build --prod` in the root folder to build a bundle for the browser
- the bundle is saved in the `dist` folder

Backend:
- Start ravestate as `ravestate_ui_new`. Spike/Activation updates will be
  served on port 4242.

