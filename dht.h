// Setup for DHT11

// DEFINE LOS PINES A UTILIZAR

// DECLARA LAS INSTANCIAS




dht11 DHT11;


/* ---------- ( FUNCION DE LECTURA DE TODOS LOS DISPOSITIVOS DHT11 ) --------*/

int read_DHT11 (int PIN){

	int read_result = DHT11.read(PIN);
	delay (1);
  			
  return read_result;
}
