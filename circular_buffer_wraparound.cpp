

class Wraparound_Queue : public Circular_Queue {

private:
	int head;
	int tail;
	int n_elem;
	// int capacity; // implied it's 256, since head and tail are int and they'll wraparound
	uint32_t* packet_history;
public:
	Wraparaound_Queue(n_elem, ) {
		if (((n_elem - 1) & n_elem) == 0) {
			head = 0;
			tail = 0;
		}
	

int ring_buffer_put(rbd_t rbd, const void *data)
{
    int err = 0;
 
    if ((rbd < RING_BUFFER_MAX) && (_ring_buffer_full(&_rb[rbd]) == 0)) {
        const size_t offset = (_rb[rbd].head & (_rb[rbd].n_elem - 1)) * _rb[rbd].s_elem;
        memcpy(&(_rb[rbd].buf[offset]), data, _rb[rbd].s_elem);
        _rb[rbd].head++;
    } else {
        err = -1;
    }
 
    return err;
}


static int _ring_buffer_full(struct ring_buffer *rb)
{
    return ((rb->head - rb->tail) == rb->n_elem) ? 1 : 0;
}
 
static int _ring_buffer_empty(struct ring_buffer *rb)
{
    return ((rb->head - rb->tail) == 0U) ? 1 : 0;
}


int ring_buffer_put(rbd_t rbd, const void *data)
{
    int err = 0;
 
    if ((rbd < RING_BUFFER_MAX) && (_ring_buffer_full(&_rb[rbd]) == 0)) {
        const size_t offset = (_rb[rbd].head & (_rb[rbd].n_elem - 1)) * _rb[rbd].s_elem;
        memcpy(&(_rb[rbd].buf[offset]), data, _rb[rbd].s_elem);
        _rb[rbd].head++;
    } else {
        err = -1;
    }
 
    return err;
}