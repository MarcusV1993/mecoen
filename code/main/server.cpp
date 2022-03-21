#include <string.h>
#include "lwip/api.h"
#include "server.h"

#include "mecoen_definitions.h"

const static char http_html_hdr[] = "HTTP/1.1 200 OK\r\nContent-type: text/html\r\n\r\n";
static char http_index_html_server[] = "\
<html>\
<body>\
<center><h1>Projeto de Graduacao em Engenharia Eletrica com Enfase em Eletronica e Sistemas</h1></center>\
<center><h1>Medidor de Consumo de Energia Eletrica Domestico de Tempo Real com Interface Via Aplicativo Web</h1></center>\
<center>Count #                                                                                                 </center>\
<center>#                                                                                                       </center>\
<center>#                                                                                                       </center>\
</body>\
</html>";

char message[100];


extern Circuit_phase phase_a;


static void http_server_netconn_serve(struct netconn *conn)
{
    struct netbuf *inbuf;
    err_t err;
    err = netconn_recv(conn, &inbuf);
    if (err == ERR_OK) 
    {
        char *output = strstr(http_index_html_server, "#");
        static int count = 0;
        sprintf(message, "%d", count++);
		strcpy(output + 1, message);
		output[strlen(message) + 1] = ';';

		output = strstr(output + 1, "#");
        sprintf(message, "Vrms = %07.2f", phase_a.voltage.rms_previous);
        strcpy(output + 1, message);
        output[strlen(message) + 1] = ';';

        output = strstr(output + 1, "#");
        sprintf(message, "Vrms = %07.2f", phase_a.current.rms_previous);
		strcpy(output + 1, message);
		output[strlen(message) + 1] = '.';

        netconn_write(conn, http_html_hdr, sizeof(http_html_hdr)-1, NETCONN_NOCOPY);
        netconn_write(conn, http_index_html_server, sizeof(http_index_html_server)-1, NETCONN_NOCOPY);
    }
    netconn_close(conn);
    netbuf_delete(inbuf);
}


void
http_server(void *pvkeys)
{
    struct netconn *conn, *newconn;
    err_t err;
    conn = netconn_new(NETCONN_TCP);
    netconn_bind(conn, NULL, 80);
    netconn_listen(conn);

    do {
        err = netconn_accept(conn, &newconn);
        if (err == ERR_OK) 
        {
            http_server_netconn_serve(newconn);
            netconn_delete(newconn);
        }
    } while(err == ERR_OK);

    netconn_close(conn);
    netconn_delete(conn);
}
