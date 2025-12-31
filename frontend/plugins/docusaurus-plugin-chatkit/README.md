# Docusaurus ChatKit Plugin

This plugin integrates a ChatKit UI component into your Docusaurus site, providing a floating chat widget that allows users to ask questions about your documentation content.

## Installation

1. Place this plugin in your Docusaurus project's `src/plugins/` directory
2. Add the plugin to your `docusaurus.config.js`:

```js
module.exports = {
  // ...
  plugins: [
    [
      './src/plugins/docusaurus-plugin-chatkit/src/index.js',
      {
        position: 'bottom-right',
        showOnMobile: true,
        sessionTimeout: 30,
        maxSelectedTextLength: 1000,
        apiBaseUrl: process.env.BACKEND_API_BASE_URL || '/api/v1',
      }
    ],
  ],
  // ...
};
```

## Configuration Options

- `position`: Where to position the chat widget ('bottom-right', 'bottom-left', 'top-right', 'top-left'). Default: 'bottom-right'
- `showOnMobile`: Whether to show the widget on mobile devices. Default: true
- `sessionTimeout`: Session timeout in minutes. Default: 30
- `maxSelectedTextLength`: Maximum length of selected text to process. Default: 1000
- `apiBaseUrl`: Base URL for the backend API. Default: '/api/v1'

## Features

- Floating chat widget available on all pages
- Text selection functionality with "Ask about this" overlay
- Session persistence across page navigations
- Responsive design for desktop and mobile
- Accessibility features (keyboard navigation, screen reader support)
- Source citations with clickable links

## Environment Variables

- `BACKEND_API_BASE_URL`: The base URL for your backend API (optional, can be set in plugin options)