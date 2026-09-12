#pragma once

// The single-page web UI, embedded at build time from src/web/www/index.html
// (see the embed_asset step in CMakeLists.txt) so an installed bibby binary
// needs no companion asset files.
extern const unsigned char web_index_html[];
extern const unsigned int  web_index_html_len;
