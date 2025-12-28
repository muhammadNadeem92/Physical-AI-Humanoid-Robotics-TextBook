const path = require('path');

/**
 * Docusaurus plugin for ChatKit UI integration
 */
module.exports = function (context, options) {
  const { siteConfig } = context;
  const config = {
    // Default configuration
    position: options.position || 'bottom-right',
    showOnMobile: options.showOnMobile !== false, // default to true
    sessionTimeout: options.sessionTimeout || 30, // in minutes
    maxSelectedTextLength: options.maxSelectedTextLength || 1000,
    apiBaseUrl: options.apiBaseUrl || siteConfig.baseUrl + 'api/v1',
    ...options,
  };

  return {
    name: 'docusaurus-plugin-chatkit',

    getClientModules() {
      return [path.resolve(__dirname, './client/chat-injector')];
    },


    injectHtmlTags() {
      return {
        // Add any necessary HTML tags
        headTags: [
          {
            tagName: 'link',
            attributes: {
              rel: 'preconnect',
              href: config.apiBaseUrl,
            },
          },
        ],
      };
    },

    async contentLoaded({ actions }) {
      const { setGlobalData } = actions;

      // Set plugin configuration as global data
      setGlobalData({
        chatkit: config,
      });
    },
  };
};